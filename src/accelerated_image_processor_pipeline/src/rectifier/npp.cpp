// Copyright 2025 TIER IV, Inc.
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

#include "accelerated_image_processor_pipeline/rectifier.hpp"
#include "utility.hpp"

#include <accelerated_image_processor_common/helper.hpp>

#include <cstring>
#include <memory>

#ifdef NPP_AVAILABLE
#include <cuda_runtime.h>
#include <npp.h>
#include <nppdefs.h>
#include <nppi.h>
#endif

namespace accelerated_image_processor::pipeline
{
#ifdef NPP_AVAILABLE
/**
 * @brief Rectifier using NPP.
 */
class NppRectifier final : public Rectifier
{
public:
  NppRectifier() : Rectifier(RectifierBackend::NPP)
  {
    cudaStreamCreateWithFlags(&stream_, cudaStreamNonBlocking);
    nppSetStream(stream_);
    
    // NppStreamContext の初期化
    // CUDA 11.4 (L4T R35) では引数で受け取る形式のみ
    nppGetStreamContext(&npp_ctx_);
    npp_ctx_.hStream = stream_;
    int device_id;
    cudaGetDevice(&device_id);
    npp_ctx_.nCudaDeviceId = device_id;
  }
  ~NppRectifier() override
  {
    if (src_host_ != nullptr) {
      cudaFreeHost(src_host_);
      src_host_ = nullptr;
    }
    if (dst_host_ != nullptr) {
      cudaFreeHost(dst_host_);
      dst_host_ = nullptr;
    }
    if (map_x_ != nullptr) {
      nppiFree(map_x_);
      map_x_ = nullptr;
    }
    if (map_y_ != nullptr) {
      nppiFree(map_y_);
      map_y_ = nullptr;
    }
    if (src_ != nullptr) {
      nppiFree(src_);
      src_ = nullptr;
    }
    if (dst_ != nullptr) {
      nppiFree(dst_);
      dst_ = nullptr;
    }
    cudaStreamDestroy(stream_);
  }

private:
  common::Image process_impl(const common::Image & image) override
  {
    common::Image result;
    result.frame_id = image.frame_id;
    result.timestamp = image.timestamp;
    result.height = image.height;
    result.width = image.width;
    result.step = image.step;
    result.encoding = image.encoding;
    result.format = image.format;

    NppiRect src_roi = {0, 0, static_cast<int>(image.width), static_cast<int>(image.height)};
    NppiSize src_size = {static_cast<int>(image.width), static_cast<int>(image.height)};
    NppiSize dst_roi_size = {static_cast<int>(image.width), static_cast<int>(image.height)};

    // ピン留めホストメモリにデータをコピー
    std::memcpy(src_host_, image.data.data(), image.data.size());

    // ホストからデバイスメモリへコピー
    CHECK_CUDA(cudaMemcpy2DAsync(
      src_, src_step_, src_host_, image.step, image.width * 3, image.height,
      cudaMemcpyHostToDevice, stream_));

    // NppStreamContext を使用した Remap
    CHECK_NPP(nppiRemap_8u_C3R_Ctx(
      src_, src_size, src_step_, src_roi, map_x_, map_x_step_, map_y_, map_y_step_, dst_, dst_step_,
      dst_roi_size, NPPI_INTER_LINEAR, npp_ctx_));

    // デバイスからホストメモリへコピー
    CHECK_CUDA(cudaMemcpy2DAsync(
      dst_host_, image.step, static_cast<const void *>(dst_),
      dst_step_,
      image.width * 3 * sizeof(Npp8u),  // in byte
      image.height, cudaMemcpyDeviceToHost, stream_));

    CHECK_CUDA(cudaStreamSynchronize(stream_));

    // 結果バッファを確保（ピン留めメモリから）
    result.data.resize(image.data.size());
    std::memcpy(result.data.data(), dst_host_, image.data.size());

    return result;
  }

  common::CameraInfo prepare_maps(const common::CameraInfo & camera_info) override
  {
    map_x_ = nppiMalloc_32f_C1(camera_info.width, camera_info.height, &map_x_step_);
    map_y_ = nppiMalloc_32f_C1(camera_info.width, camera_info.height, &map_y_step_);

    src_ = nppiMalloc_8u_C3(camera_info.width, camera_info.height, &src_step_);
    dst_ = nppiMalloc_8u_C3(camera_info.width, camera_info.height, &dst_step_);

    // ピン留めホストメモリの確保
    CHECK_CUDA(cudaMallocHost(
      reinterpret_cast<void **>(&src_host_), camera_info.width * camera_info.height * 3));
    CHECK_CUDA(cudaMallocHost(
      reinterpret_cast<void **>(&dst_host_), camera_info.width * camera_info.height * 3));

    float * map_x = new float[camera_info.width * camera_info.height];
    float * map_y = new float[camera_info.width * camera_info.height];

    common::CameraInfo camera_info_rect = compute_maps(camera_info, map_x, map_y, alpha());

    CHECK_CUDA(cudaMemcpy2DAsync(
      map_x_, map_x_step_, map_x, camera_info.width * sizeof(float),
      camera_info.width * sizeof(float), camera_info.height, cudaMemcpyHostToDevice, stream_));
    CHECK_CUDA(cudaMemcpy2DAsync(
      map_y_, map_y_step_, map_y, camera_info.width * sizeof(float),
      camera_info.width * sizeof(float), camera_info.height, cudaMemcpyHostToDevice, stream_));

    CHECK_CUDA(cudaStreamSynchronize(stream_));

    delete[] map_x;
    delete[] map_y;

    return camera_info_rect;
  }

  Npp32f * map_x_{nullptr};
  Npp32f * map_y_{nullptr};
  int map_x_step_{0};
  int map_y_step_{0};
  Npp8u * src_{nullptr};
  Npp8u * dst_{nullptr};
  int src_step_{0};
  int dst_step_{0};
  uint8_t * src_host_{nullptr};
  uint8_t * dst_host_{nullptr};
  NppStreamContext npp_ctx_{};
  cudaStream_t stream_;
};

std::unique_ptr<Rectifier> make_npp_rectifier()
{
  return std::make_unique<NppRectifier>();
}
#else
std::unique_ptr<Rectifier> make_npp_rectifier()
{
  return nullptr;
}
#endif  // NPP_AVAILABLE
}  // namespace accelerated_image_processor::pipeline
