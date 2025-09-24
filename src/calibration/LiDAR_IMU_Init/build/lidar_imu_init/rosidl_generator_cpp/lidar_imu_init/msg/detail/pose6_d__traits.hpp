// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from lidar_imu_init:msg/Pose6D.idl
// generated code does not contain a copyright notice

#ifndef LIDAR_IMU_INIT__MSG__DETAIL__POSE6_D__TRAITS_HPP_
#define LIDAR_IMU_INIT__MSG__DETAIL__POSE6_D__TRAITS_HPP_

#include "lidar_imu_init/msg/detail/pose6_d__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<lidar_imu_init::msg::Pose6D>()
{
  return "lidar_imu_init::msg::Pose6D";
}

template<>
inline const char * name<lidar_imu_init::msg::Pose6D>()
{
  return "lidar_imu_init/msg/Pose6D";
}

template<>
struct has_fixed_size<lidar_imu_init::msg::Pose6D>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<lidar_imu_init::msg::Pose6D>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<lidar_imu_init::msg::Pose6D>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // LIDAR_IMU_INIT__MSG__DETAIL__POSE6_D__TRAITS_HPP_
