cmake_minimum_required(VERSION 3.10)
project(pointcloud_preprocess)

# 使用C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# 创建组件库
add_library(pointcloud_preprocess_component SHARED
  src/pointcloud_transformer.cpp
)

target_compile_definitions(pointcloud_preprocess_component
  PRIVATE "POINTCLOUD_PREPROCESS_BUILDING_DLL")

target_include_directories(pointcloud_preprocess_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(pointcloud_preprocess_component
  rclcpp
  rclcpp_components
  sensor_msgs
  pcl_conversions
  pcl_ros
  tf2_ros
  tf2_eigen
  tf2_sensor_msgs
  tf2_geometry_msgs
)

# 注册组件
rclcpp_components_register_node(pointcloud_preprocess_component
  PLUGIN "pointcloud_preprocess::PointCloudTransformerNode"
  EXECUTABLE pointcloud_transformer_node
)

# 安装
install(
  TARGETS pointcloud_preprocess_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

# 安装README
install(FILES
  README.md
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package() 