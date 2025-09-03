cmake_minimum_required(VERSION 3.8)
project(remote_controller_simulator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(bot_msg REQUIRED)


# include directories
include_directories(include)

# add executable
add_executable(remote_controller_simulator_node src/remote_controller_simulator_simple.cpp)
ament_target_dependencies(remote_controller_simulator_node rclcpp std_msgs bot_msg)

# specify include directories for this target
target_include_directories(remote_controller_simulator_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# install targets
install(TARGETS
  remote_controller_simulator_node
  DESTINATION lib/${PROJECT_NAME})

# install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()