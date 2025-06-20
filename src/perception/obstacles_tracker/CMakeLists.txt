cmake_minimum_required(VERSION 3.8)
project(obstacles_tracker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(bot_msg REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

# 添加主节点可执行文件
add_executable(obstacles_tracker_node 
  src/obstacles_tracker_node.cpp
  src/obstacles_tracker.cpp
  src/kalman_filter.cpp
)

# 添加测试发布器可执行文件
add_executable(test_publisher_node
  src/test_publisher.cpp
)

# 链接依赖库
target_link_libraries(obstacles_tracker_node
  ${EIGEN3_LIBRARIES}
)

# 依赖项
ament_target_dependencies(obstacles_tracker_node
  rclcpp
  std_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  bot_msg
)

# 添加测试发布器依赖项
ament_target_dependencies(test_publisher_node
  rclcpp
  std_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  bot_msg
)

# 安装可执行文件
install(TARGETS
  obstacles_tracker_node
  test_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装头文件
install(DIRECTORY
  include/
  DESTINATION include/
)

# 安装启动文件
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# 安装配置文件
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package() 