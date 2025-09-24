// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from lidar_imu_init:msg/States.idl
// generated code does not contain a copyright notice

#ifndef LIDAR_IMU_INIT__MSG__DETAIL__STATES__BUILDER_HPP_
#define LIDAR_IMU_INIT__MSG__DETAIL__STATES__BUILDER_HPP_

#include "lidar_imu_init/msg/detail/states__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace lidar_imu_init
{

namespace msg
{

namespace builder
{

class Init_States_cov
{
public:
  explicit Init_States_cov(::lidar_imu_init::msg::States & msg)
  : msg_(msg)
  {}
  ::lidar_imu_init::msg::States cov(::lidar_imu_init::msg::States::_cov_type arg)
  {
    msg_.cov = std::move(arg);
    return std::move(msg_);
  }

private:
  ::lidar_imu_init::msg::States msg_;
};

class Init_States_gravity
{
public:
  explicit Init_States_gravity(::lidar_imu_init::msg::States & msg)
  : msg_(msg)
  {}
  Init_States_cov gravity(::lidar_imu_init::msg::States::_gravity_type arg)
  {
    msg_.gravity = std::move(arg);
    return Init_States_cov(msg_);
  }

private:
  ::lidar_imu_init::msg::States msg_;
};

class Init_States_bias_acc
{
public:
  explicit Init_States_bias_acc(::lidar_imu_init::msg::States & msg)
  : msg_(msg)
  {}
  Init_States_gravity bias_acc(::lidar_imu_init::msg::States::_bias_acc_type arg)
  {
    msg_.bias_acc = std::move(arg);
    return Init_States_gravity(msg_);
  }

private:
  ::lidar_imu_init::msg::States msg_;
};

class Init_States_bias_gyr
{
public:
  explicit Init_States_bias_gyr(::lidar_imu_init::msg::States & msg)
  : msg_(msg)
  {}
  Init_States_bias_acc bias_gyr(::lidar_imu_init::msg::States::_bias_gyr_type arg)
  {
    msg_.bias_gyr = std::move(arg);
    return Init_States_bias_acc(msg_);
  }

private:
  ::lidar_imu_init::msg::States msg_;
};

class Init_States_vel_end
{
public:
  explicit Init_States_vel_end(::lidar_imu_init::msg::States & msg)
  : msg_(msg)
  {}
  Init_States_bias_gyr vel_end(::lidar_imu_init::msg::States::_vel_end_type arg)
  {
    msg_.vel_end = std::move(arg);
    return Init_States_bias_gyr(msg_);
  }

private:
  ::lidar_imu_init::msg::States msg_;
};

class Init_States_pos_end
{
public:
  explicit Init_States_pos_end(::lidar_imu_init::msg::States & msg)
  : msg_(msg)
  {}
  Init_States_vel_end pos_end(::lidar_imu_init::msg::States::_pos_end_type arg)
  {
    msg_.pos_end = std::move(arg);
    return Init_States_vel_end(msg_);
  }

private:
  ::lidar_imu_init::msg::States msg_;
};

class Init_States_rot_end
{
public:
  explicit Init_States_rot_end(::lidar_imu_init::msg::States & msg)
  : msg_(msg)
  {}
  Init_States_pos_end rot_end(::lidar_imu_init::msg::States::_rot_end_type arg)
  {
    msg_.rot_end = std::move(arg);
    return Init_States_pos_end(msg_);
  }

private:
  ::lidar_imu_init::msg::States msg_;
};

class Init_States_header
{
public:
  Init_States_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_States_rot_end header(::lidar_imu_init::msg::States::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_States_rot_end(msg_);
  }

private:
  ::lidar_imu_init::msg::States msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::lidar_imu_init::msg::States>()
{
  return lidar_imu_init::msg::builder::Init_States_header();
}

}  // namespace lidar_imu_init

#endif  // LIDAR_IMU_INIT__MSG__DETAIL__STATES__BUILDER_HPP_
