// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from lidar_imu_init:msg/States.idl
// generated code does not contain a copyright notice

#ifndef LIDAR_IMU_INIT__MSG__DETAIL__STATES__STRUCT_H_
#define LIDAR_IMU_INIT__MSG__DETAIL__STATES__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'rot_end'
// Member 'pos_end'
// Member 'vel_end'
// Member 'bias_gyr'
// Member 'bias_acc'
// Member 'gravity'
// Member 'cov'
#include "rosidl_runtime_c/primitives_sequence.h"

// Struct defined in msg/States in the package lidar_imu_init.
typedef struct lidar_imu_init__msg__States
{
  std_msgs__msg__Header header;
  rosidl_runtime_c__double__Sequence rot_end;
  rosidl_runtime_c__double__Sequence pos_end;
  rosidl_runtime_c__double__Sequence vel_end;
  rosidl_runtime_c__double__Sequence bias_gyr;
  rosidl_runtime_c__double__Sequence bias_acc;
  rosidl_runtime_c__double__Sequence gravity;
  rosidl_runtime_c__double__Sequence cov;
} lidar_imu_init__msg__States;

// Struct for a sequence of lidar_imu_init__msg__States.
typedef struct lidar_imu_init__msg__States__Sequence
{
  lidar_imu_init__msg__States * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} lidar_imu_init__msg__States__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // LIDAR_IMU_INIT__MSG__DETAIL__STATES__STRUCT_H_
