cmake_minimum_required(VERSION 3.5)
project(hc_gnss)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17 (required for std::filesystem)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set build type to Release
set(CMAKE_BUILD_TYPE "Release")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(bot_msg REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(include)

add_executable(hc_gnss_node src/hc_gnss.cpp src/gnss_serial.cpp)

# Specify libraries to link against
target_link_libraries(hc_gnss_node
  stdc++fs
)

# Install executable
install(TARGETS
  hc_gnss_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
  launch 
  config
  DESTINATION share/${PROJECT_NAME}
)

# Export dependencies and package
ament_target_dependencies(hc_gnss_node
  bot_msg
  rclcpp
  geometry_msgs
  sensor_msgs
  tf2
  tf2_geometry_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()