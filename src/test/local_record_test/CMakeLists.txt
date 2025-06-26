cmake_minimum_required(VERSION 3.5)
project(local_record_test)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(bot_msg REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include)

add_executable(local_record_test_node src/local_record_test.cpp)

# Use ament_target_dependencies to link required dependencies
ament_target_dependencies(local_record_test_node
  bot_msg
  rclcpp
  std_msgs
  rosgraph_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(TARGETS local_record_test_node
  DESTINATION lib/${PROJECT_NAME}
)