// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from lidar_imu_init:msg/Pose6D.idl
// generated code does not contain a copyright notice

#ifndef LIDAR_IMU_INIT__MSG__DETAIL__POSE6_D__STRUCT_H_
#define LIDAR_IMU_INIT__MSG__DETAIL__POSE6_D__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in msg/Pose6D in the package lidar_imu_init.
typedef struct lidar_imu_init__msg__Pose6D
{
  double offset_time;
  double acc[3];
  double gyr[3];
  double vel[3];
  double pos[3];
  double rot[9];
} lidar_imu_init__msg__Pose6D;

// Struct for a sequence of lidar_imu_init__msg__Pose6D.
typedef struct lidar_imu_init__msg__Pose6D__Sequence
{
  lidar_imu_init__msg__Pose6D * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} lidar_imu_init__msg__Pose6D__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // LIDAR_IMU_INIT__MSG__DETAIL__POSE6_D__STRUCT_H_
