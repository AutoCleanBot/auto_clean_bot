cmake_minimum_required(VERSION 3.5)
project(innovusion)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # add_compile_options(-Wall -Wextra -Wpedantic)
  add_compile_options(-Wno-format-extra-args -Wno-format)
endif()

if(DEFINED ENV{ARCH_TAG})
  if(($ENV{ARCH_TAG} STREQUAL "-aarch64")
      OR($ENV{ARCH_TAG} STREQUAL "-arm"))
	set(CMAKE_LIBRARY_PATH "/usr/lib/aarch64-linux-gnu" ${CMAKE_LIBRARY_PATH})
  endif()
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(message_filters REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/InnovusionPacket.msg"
  "msg/InnovusionScan.msg"
  DEPENDENCIES std_msgs
)

set(INNO_SDK_PATH
  src/inno_sdk/
)

if(DEFINED ENV{ARCH_TAG})
  # in cross compile, redirect pcl lib path
  if(($ENV{ARCH_TAG} STREQUAL "-aarch64")
      OR($ENV{ARCH_TAG} STREQUAL "-arm"))
      set(PCL_LIBRARIES
          /lib/aarch64-linux-gnu
      )
      find_library(PCL_ROS_TF pcl_ros_tf /opt/ros/$ENV{ROS_DISTRO}/lib)
      find_library(PCL_COMMON pcl_common /lib/aarch64-linux-gnu)
      list(APPEND PCL_LIBRARIES ${PCL_ROS_TF} ${PCL_COMMON})
  endif()
endif()

# add PCL include PATH
list(APPEND PCL_INCLUDE_DIRS /opt/ros/$ENV{ROS_DISTRO}/include)

# publisher
add_executable(publisher
               src/node/publisher.cc
               src/node/driver/driver_lidar.cc
               ${INNO_SDK_PATH}/src/sdk_common/converter/cframe_converter.cpp
)

rosidl_target_interfaces(publisher ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_include_directories(publisher
  PRIVATE
  src/node/driver
  src/thirdparty
  ${INNO_SDK_PATH}
  ${INNO_SDK_PATH}/apps
  ${INNO_SDK_PATH}/src
  ${PCL_INCLUDE_DIRS}
)

target_link_directories(publisher
  PRIVATE
  ${INNO_SDK_PATH}/lib
)

target_link_libraries(publisher
  innolidarsdkclient
  innolidarsdkcommon
  innolidarutils
  ${PCL_LIBRARIES}
  pcap
)

ament_target_dependencies(publisher
  rclcpp
  PCL
  std_msgs
  sensor_msgs
  message_filters
  pcl_msgs
)

if(APPLE)
  add_custom_command(TARGET publisher
    POST_BUILD COMMAND
    ${CMAKE_INSTALL_NAME_TOOL} -change libinnolidarsdk.so.0 @rpath/libinnolidarsdk.so publisher
    COMMAND
    ${CMAKE_INSTALL_NAME_TOOL} -add_rpath "@executable_path/" publisher
  )
else()
  set_target_properties(publisher PROPERTIES
    BUILD_WITH_INSTALL_RPATH FALSE
    LINK_FLAGS "-Wl,-rpath,$ORIGIN/")
endif()

# install group
install(TARGETS
  publisher
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY
  ${INNO_SDK_PATH}/lib/
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
