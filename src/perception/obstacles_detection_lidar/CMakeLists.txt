cmake_minimum_required(VERSION 3.5)
project(obstacles_detection_lidar)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
# 针对 GCC 编译器设置优化选项
if(CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O2 -march=native -mtune=native")
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set build type to Debug
set(CMAKE_BUILD_TYPE "Release")

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rosidl_typesupport_fastrtps_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(bot_msg REQUIRED)
# find_package(conti_radar REQUIRED)
# find_package(pcl_ros REQUIRED)
# find_package(pcl_conversions REQUIRED)
find_package(tf2_ros REQUIRED)

# Include directories
include_directories(
  include
  ${tf2_ros_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${pcl_ros_INCLUDE_DIRS}

)

# Create your node executable
add_executable(obstacles_detection_lidar_node src/obstacles_detection_lidar.cpp)

# Add PCL definitions and link directories
add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})



# Specify libraries to link against
target_link_libraries(obstacles_detection_lidar_node
  ${rclcpp_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${PCL_LIBRARIES}
  ${tf2_ros_LIBRARIES}
  yaml-cpp
)

# Ensure the generated headers are included in the build
target_include_directories(obstacles_detection_lidar_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
  $<INSTALL_INTERFACE:include>
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Install executable
install(TARGETS
  obstacles_detection_lidar_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
launch DESTINATION share/${PROJECT_NAME})

# Export dependencies and package
ament_target_dependencies(obstacles_detection_lidar_node
  rosidl_typesupport_fastrtps_cpp
  rclcpp
  sensor_msgs
  visualization_msgs
  bot_msg
  # conti_radar
)
# 导出依赖
ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(rosidl_default_generators)
ament_package()
