cmake_minimum_required(VERSION 3.5)
project(data_recorder)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
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
find_package(rclcpp REQUIRED)
find_package(bot_msg REQUIRED)

include_directories(include)

# Add executable
add_executable(data_recorder_node src/data_recorder.cpp)

# Specify libraries to link against
target_link_libraries(data_recorder_node
  stdc++fs
)

# Install executable
install(TARGETS
  data_recorder_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch directory
install(DIRECTORY 
  launch 
  DESTINATION share/${PROJECT_NAME}
)

# Install config directory
install(DIRECTORY 
  config 
  DESTINATION share/${PROJECT_NAME}
)

# Export dependencies
ament_target_dependencies(data_recorder_node
  rclcpp
  bot_msg
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