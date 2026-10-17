# accelerated_image_processor_rectify

A rectify-only ROS 2 node built on [`accelerated_image_processor_pipeline`](../accelerated_image_processor_pipeline/README.md).

It wraps `pipeline::Rectifier` (NPP > OpenCV CUDA > OpenCV CPU, selected at build time) with
the same interface as `image_proc::RectifyNode`, so it can be used as a drop-in replacement
of `image_proc` rectification on Jetson.

Unlike `ImgProcNode` of `accelerated_image_processor_ros`, this node does not compress images,
so it depends only on `accelerated_image_processor_common` and `accelerated_image_processor_pipeline`
(no Jetson Multimedia API / NVJPEG / FFmpeg).

## Requirements

Verified on Jetson Orin Nano (L4T R35.6 / JetPack 5) with the plain `ros:humble-ros-base` container
and the host CUDA 11.4 mounted. No Jetson Multimedia API, `/etc/nv_tegra_release`, TurboJPEG nor
FFmpeg are required.

| Item                          | Why / symptom if missing                                                                                                                        |
| ----------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------- |
| ROS 2 Humble, `libopencv-dev` | Build dependencies                                                                                                                              |
| CUDA toolkit (nvcc + NPP)     | `accelerated_image_processor_pipeline` requires the CUDA language. Build fails with `Failed to find nvcc`                                      |
| `nvcc` in `PATH` at build     | JetPack does not add `/usr/local/cuda/bin` to `PATH`. Alternatively pass `-DCMAKE_CUDA_COMPILER=/usr/local/cuda/bin/nvcc`                     |
| CUDA libs in the loader path  | Node dies on start with `libnppc.so.11: cannot open shared object file`                                                                       |
| `--runtime nvidia` (container) | With `NVIDIA_VISIBLE_DEVICES=all` and `NVIDIA_DRIVER_CAPABILITIES=all`. Otherwise `cudaErrorInsufficientDriver` (host `libcuda` is not mounted) |

Minimal example in a container:

```shell
# on the Jetson host (CUDA 11.4 for JetPack 5)
docker run -it --runtime nvidia \
  -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all \
  -v /usr/local/cuda-11.4:/usr/local/cuda-11.4:ro \
  ros:humble-ros-base bash

# in the container (repository cloned into ~/ros2_ws/src)
apt-get update && apt-get install -y libopencv-dev
source /opt/ros/humble/setup.bash
export PATH=/usr/local/cuda-11.4/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda-11.4/lib64:$LD_LIBRARY_PATH  # also needed at runtime
cd ~/ros2_ws
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
  --packages-up-to accelerated_image_processor_rectify
```

On start the node logs `Rectifier backend: NPP (GPU)`. If it says `OpenCV (CPU)`, NPP was not
found at build time.

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
