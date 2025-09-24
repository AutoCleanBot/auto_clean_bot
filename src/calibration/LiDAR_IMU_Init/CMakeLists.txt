cmake_minimum_required(VERSION 3.5)
project(lidar_imu_init)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

message("Current CPU architecture: ${CMAKE_SYSTEM_PROCESSOR}")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)")
  include(ProcessorCount)
  ProcessorCount(N)
  message("Processer number:  ${N}")
  if(N GREATER 4)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=3)
    message("core for MP: 3")
  elseif(N GREATER 3)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=2)
    message("core for MP: 2")
  else()
    add_definitions(-DMP_PROC_NUM=1)
  endif()
else()
  add_definitions(-DMP_PROC_NUM=1)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED COMPONENTS common io filters)
find_package(Ceres REQUIRED)
find_package(OpenMP QUIET)
find_package(Python3 COMPONENTS Development REQUIRED)
find_path(MATPLOTLIB_CPP_INCLUDE_DIRS "matplotlibcpp.h")
find_package(rosidl_default_generators REQUIRED)

if(NOT MATPLOTLIB_CPP_INCLUDE_DIRS)
  set(MATPLOTLIB_CPP_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/include")
endif()

# Ensure rosidl installation check sees the package as an interface provider
set(${PROJECT_NAME}_MEMBER_OF_GROUPS "rosidl_interface_packages")

if(OpenMP_CXX_FOUND)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Pose6D.msg"
  "msg/States.msg"
  DEPENDENCIES std_msgs
  SKIP_GROUP_MEMBERSHIP_CHECK
)

ament_export_dependencies(rosidl_default_runtime)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${Python3_INCLUDE_DIRS}
  ${MATPLOTLIB_CPP_INCLUDE_DIRS}
)

add_executable(li_init
  src/laserMapping.cpp
  include/ikd-Tree/ikd_Tree.cpp
  include/LI_init/LI_init.cpp
  src/preprocess.cpp
)

target_include_directories(li_init PRIVATE ${Python3_INCLUDE_DIRS})

ament_target_dependencies(li_init
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_conversions
  ament_index_cpp
)

target_link_libraries(li_init
  ${PCL_LIBRARIES}
  ${Python3_LIBRARIES}
  ${CERES_LIBRARIES}
)

if(OpenMP_CXX_FOUND)
  target_link_libraries(li_init OpenMP::OpenMP_CXX)
endif()

rosidl_target_interfaces(li_init
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS li_init
  DESTINATION lib/${PROJECT_NAME})

ament_package()
