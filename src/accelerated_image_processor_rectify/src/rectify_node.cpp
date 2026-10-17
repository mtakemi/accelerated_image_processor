// Copyright 2026 masafumi
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// image_proc の RectifyNode 互換 (image + camera_info -> image_rect) の rectify 専用ノード。
// rectify 本体は accelerated_image_processor_pipeline (NPP > OpenCV CUDA > CPU) を利用する。

#include <accelerated_image_processor_common/datatype.hpp>
#include <accelerated_image_processor_pipeline/builder.hpp>
#include <accelerated_image_processor_pipeline/rectifier.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <sensor_msgs/distortion_models.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

namespace accelerated_image_processor::rectify
{
namespace
{
int64_t to_nanoseconds(const builtin_interfaces::msg::Time & stamp)
{
  return rclcpp::Time(stamp).nanoseconds();
}

common::DistortionModel to_distortion_model(const std::string & model)
{
  if (model == sensor_msgs::distortion_models::PLUMB_BOB) {
    return common::DistortionModel::PLUMB_BOB;
  } else if (model == sensor_msgs::distortion_models::EQUIDISTANT) {
    return common::DistortionModel::EQUIDISTANT;
  } else if (model == sensor_msgs::distortion_models::RATIONAL_POLYNOMIAL) {
    return common::DistortionModel::RATIONAL_POLYNOMIAL;
  }
  throw std::runtime_error("Unsupported distortion model: " + model);
}

std::string to_ros_distortion_model(common::DistortionModel model)
{
  switch (model) {
    case common::DistortionModel::EQUIDISTANT:
      return sensor_msgs::distortion_models::EQUIDISTANT;
    case common::DistortionModel::RATIONAL_POLYNOMIAL:
      return sensor_msgs::distortion_models::RATIONAL_POLYNOMIAL;
    case common::DistortionModel::PLUMB_BOB:
    default:
      return sensor_msgs::distortion_models::PLUMB_BOB;
  }
}

const char * to_string(pipeline::RectifierBackend backend)
{
  switch (backend) {
    case pipeline::RectifierBackend::NPP:
      return "NPP (GPU)";
    case pipeline::RectifierBackend::OPENCV_CUDA:
      return "OpenCV CUDA (GPU)";
    case pipeline::RectifierBackend::CPU:
    default:
      return "OpenCV (CPU)";
  }
}
}  // namespace

class RectifyNode final : public rclcpp::Node
{
public:
  explicit RectifyNode(const rclcpp::NodeOptions & options) : Node("rectify_node", options)
  {
    rectifier_ = pipeline::create_rectifier();
    // alpha < 0: image_proc 互換 (camera_info の P を使用), 0..1: getOptimalNewCameraMatrix
    rectifier_->parameter_value<double>("alpha") = this->declare_parameter<double>("alpha", -1.0);
    RCLCPP_INFO(this->get_logger(), "Rectifier backend: %s", to_string(rectifier_->backend()));

    image_rect_pub_ = this->create_publisher<sensor_msgs::msg::Image>("image_rect", 10);
    info_rect_pub_ =
      this->create_publisher<sensor_msgs::msg::CameraInfo>("image_rect/camera_info", 10);

    // NOTE: 数 MB の画像を best effort で受けると、DDS のフラグメントが 1 つ欠けただけで
    // フレームごと破棄されるため、デフォルトは reliable (image_proc と同じ)。
    // publisher が best effort の場合のみ use_sensor_data_qos:=true にする
    const bool use_sensor_data_qos = this->declare_parameter<bool>("use_sensor_data_qos", false);
    const rclcpp::QoS input_qos =
      use_sensor_data_qos ? rclcpp::QoS(rclcpp::SensorDataQoS()) : rclcpp::QoS(1).reliable();

    info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      "camera_info", input_qos,
      [this](const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) { on_camera_info(msg); });
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "image", input_qos,
      [this](const sensor_msgs::msg::Image::ConstSharedPtr msg) { on_image(msg); });
  }

private:
  void on_camera_info(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg)
  {
    // NOTE: Rectifier::set_camera_info() は呼ぶたびに GPU バッファを確保し直す (解放しない) ため、
    // 最初の 1 回だけ map を計算する
    if (rectifier_->is_ready()) {
      return;
    }

    common::CameraInfo info;
    info.frame_id = msg->header.frame_id;
    info.timestamp = to_nanoseconds(msg->header.stamp);
    info.height = msg->height;
    info.width = msg->width;
    info.distortion_model = to_distortion_model(msg->distortion_model);
    info.d = msg->d;
    info.k = msg->k;
    info.r = msg->r;
    info.p = msg->p;
    info.binning_x = msg->binning_x;
    info.binning_y = msg->binning_y;
    info.roi = common::Roi{
      msg->roi.x_offset, msg->roi.y_offset, msg->roi.width, msg->roi.height, msg->roi.do_rectify};

    rectifier_->set_camera_info(info);
    RCLCPP_INFO(
      this->get_logger(), "Rectification maps prepared (%ux%u)", info.width, info.height);
  }

  void on_image(const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    if (!rectifier_->is_ready()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000, "Waiting for camera_info...");
      return;
    }

    common::ImageEncoding encoding;
    if (msg->encoding == sensor_msgs::image_encodings::RGB8) {
      encoding = common::ImageEncoding::RGB;
    } else if (msg->encoding == sensor_msgs::image_encodings::BGR8) {
      encoding = common::ImageEncoding::BGR;
    } else {
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Unsupported encoding: %s (only rgb8/bgr8)", msg->encoding.c_str());
      return;
    }

    // NPP rectifier は camera_info サイズ・行パディング無しの 3ch 画像前提でバッファを確保している
    const auto & info = rectifier_->camera_info().value();
    if (msg->width != info.width || msg->height != info.height || msg->step != msg->width * 3) {
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Image (%ux%u, step=%u) does not match camera_info (%ux%u)", msg->width, msg->height,
        msg->step, info.width, info.height);
      return;
    }

    common::Image image;
    image.frame_id = msg->header.frame_id;
    image.timestamp = to_nanoseconds(msg->header.stamp);
    image.height = msg->height;
    image.width = msg->width;
    image.step = msg->step;
    image.encoding = encoding;
    image.format = common::ImageFormat::RAW;
    image.is_bigendian = false;
    image.data = msg->data;

    auto rectified = rectifier_->process(image);
    if (!rectified) {
      return;
    }

    auto out = std::make_unique<sensor_msgs::msg::Image>();
    out->header = msg->header;
    out->height = rectified->height;
    out->width = rectified->width;
    out->encoding = msg->encoding;
    out->is_bigendian = false;
    out->step = rectified->step;
    out->data = std::move(rectified->data);
    image_rect_pub_->publish(std::move(out));

    auto info_out = std::make_unique<sensor_msgs::msg::CameraInfo>();
    info_out->header = msg->header;
    info_out->height = info.height;
    info_out->width = info.width;
    info_out->distortion_model = to_ros_distortion_model(info.distortion_model);
    info_out->d = info.d;
    info_out->k = info.k;
    info_out->r = info.r;
    info_out->p = info.p;
    info_out->binning_x = info.binning_x;
    info_out->binning_y = info.binning_y;
    info_rect_pub_->publish(std::move(info_out));
  }

  std::unique_ptr<pipeline::Rectifier> rectifier_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_rect_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_rect_pub_;
};
}  // namespace accelerated_image_processor::rectify

RCLCPP_COMPONENTS_REGISTER_NODE(accelerated_image_processor::rectify::RectifyNode)
