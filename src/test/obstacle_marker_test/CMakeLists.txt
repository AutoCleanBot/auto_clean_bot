cmake_minimum_required(VERSION 3.8)
project(obstacle_marker_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(unique_identifier_msgs REQUIRED)
find_package(Boost REQUIRED)

# include directories
include_directories(include)

# add executable
add_executable(obstacle_marker_test_node src/obstacle_marker_test_node.cpp src/debug_object.cpp)

ament_target_dependencies(obstacle_marker_test_node
  rclcpp
  std_msgs
  geometry_msgs
  visualization_msgs
  unique_identifier_msgs
)

# Link Boost libraries
target_link_libraries(obstacle_marker_test_node ${Boost_LIBRARIES})

# install targets
install(TARGETS
  obstacle_marker_test_node
  DESTINATION lib/${PROJECT_NAME}
)

# install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
