cmake_minimum_required(VERSION 3.5)
project(can1_bus)

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
find_package(rclcpp REQUIRED)

include_directories(include)

add_executable(can1_bus_node src/can_bus1.cpp)

# Use ament_target_dependencies to link required dependencies
ament_target_dependencies(can1_bus_node
  rclcpp
  std_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

install(TARGETS can1_bus_node
  DESTINATION lib/${PROJECT_NAME}
)