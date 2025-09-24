// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from lidar_imu_init:msg/States.idl
// generated code does not contain a copyright notice

#ifndef LIDAR_IMU_INIT__MSG__DETAIL__STATES__TRAITS_HPP_
#define LIDAR_IMU_INIT__MSG__DETAIL__STATES__TRAITS_HPP_

#include "lidar_imu_init/msg/detail/states__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<lidar_imu_init::msg::States>()
{
  return "lidar_imu_init::msg::States";
}

template<>
inline const char * name<lidar_imu_init::msg::States>()
{
  return "lidar_imu_init/msg/States";
}

template<>
struct has_fixed_size<lidar_imu_init::msg::States>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<lidar_imu_init::msg::States>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<lidar_imu_init::msg::States>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // LIDAR_IMU_INIT__MSG__DETAIL__STATES__TRAITS_HPP_
