# accelerated_image_processor_rectify

A rectify-only ROS 2 node built on [`accelerated_image_processor_pipeline`](../accelerated_image_processor_pipeline/README.md).

It wraps `pipeline::Rectifier` (NPP > OpenCV CUDA > OpenCV CPU, selected at build time) with
the same interface as `image_proc::RectifyNode`, so it can be used as a drop-in replacement
of `image_proc` rectification on Jetson.

Unlike `ImgProcNode` of `accelerated_image_processor_ros`, this node does not compress images,
so it depends only on `accelerated_image_processor_common` and `accelerated_image_processor_pipeline`
(no Jetson Multimedia API / NVJPEG / FFmpeg).

## Launch

```shell
ros2 launch accelerated_image_processor_rectify rectify.launch.xml
```

## I/O Topics

| Direction | Name                     | Type                     | Description                        |
| --------- | ------------------------ | ------------------------ | ---------------------------------- |
| Input     | `image`                  | `sensor_msgs/Image`      | Raw image (`rgb8` or `bgr8`)       |
| Input     | `camera_info`            | `sensor_msgs/CameraInfo` | Camera info of the raw image       |
| Output    | `image_rect`             | `sensor_msgs/Image`      | Rectified image                    |
| Output    | `image_rect/camera_info` | `sensor_msgs/CameraInfo` | Camera info of the rectified image |

## Parameters

| Name                  | Type     | Default | Description                                                                                                                            |
| --------------------- | -------- | ------- | -------------------------------------------------------------------------------------------------------------------------------------- |
| `alpha`               | `double` | `-1.0`  | `< 0`: use R and P of the input `camera_info` (same as `image_proc`). `0..1`: new camera matrix by `cv::getOptimalNewCameraMatrix`. |
| `use_sensor_data_qos` | `bool`   | `false` | Subscribe inputs with best effort QoS. Use only if the publisher is best effort.                                                      |

## Limitations

- Only 3-channel 8-bit images (`rgb8`, `bgr8`) without row padding are supported.
- The rectification maps are computed from the first `camera_info` only.
