cmake_minimum_required(VERSION 3.16)
project(accelerated_image_processor_rectify)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wunused-function)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED src/rectify_node.cpp)

rclcpp_components_register_node(
  ${PROJECT_NAME} PLUGIN "accelerated_image_processor::rectify::RectifyNode"
  EXECUTABLE ${PROJECT_NAME}_node)

ament_auto_package(INSTALL_TO_SHARE launch)
