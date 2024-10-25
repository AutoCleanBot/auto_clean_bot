cmake_minimum_required(VERSION 3.8)
project(lane_detection)


# CUDA and TensorRT support
# set(CMAKE_CUDA_ARCHITECTURES 60 61 62 70 72 75 86 89 90)
# set(CMAKE_CUDA_COMPILER /usr/local/cuda/bin/nvcc)
# option(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
# set(CUDA_INCLUDE_DIRS /usr/local/cuda/include)
# set(CUDA_LIBRARIES /usr/local/cuda/lib64)
# set(TENSORRT_INCLUDE_DIRS /usr/include/aarch64-linux-gnu/)
# set(TENSORRT_LIBRARY_DIRS /usr/lib/aarch64-linux-gnu/)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(CUDA REQUIRED)

# Find TensorRT
find_path(TENSORRT_INCLUDE_DIR NvInfer.h
  HINTS ${TENSORRT_ROOT} ${CUDA_TOOLKIT_ROOT_DIR}
  PATH_SUFFIXES include)
find_library(TENSORRT_LIBRARY nvinfer
  HINTS ${TENSORRT_ROOT} ${CUDA_TOOLKIT_ROOT_DIR}
  PATH_SUFFIXES lib lib64)

# Include directories
include_directories(
  include
  ${CUDA_INCLUDE_DIRS}
  ${TENSORRT_INCLUDE_DIR}
)

# Create the ufld_detector library
add_library(ufld_detector
  src/ufld_detector.cpp
)
target_include_directories(ufld_detector PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(ufld_detector
  ${CUDA_LIBRARIES}
  ${TENSORRT_LIBRARY}
  ${OpenCV_LIBS}
)
ament_target_dependencies(ufld_detector
  rclcpp
  sensor_msgs
  cv_bridge
)

# Create the ROS2 node executable
add_executable(ufld_node src/ufld_node.cpp)
target_link_libraries(ufld_node
  ufld_detector
)
ament_target_dependencies(ufld_node
  rclcpp
  sensor_msgs
  cv_bridge
)

# Install targets
install(TARGETS
  ufld_detector
  ufld_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install include directory
install(DIRECTORY include/
  DESTINATION include/
)

# Install launch files
install(DIRECTORY
  launch config
  DESTINATION share/${PROJECT_NAME}
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Export dependencies
ament_export_include_directories(include)
ament_export_libraries(ufld_detector)
ament_export_dependencies(
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
)

ament_package()