// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from lidar_imu_init:msg/Pose6D.idl
// generated code does not contain a copyright notice

#ifndef LIDAR_IMU_INIT__MSG__DETAIL__POSE6_D__BUILDER_HPP_
#define LIDAR_IMU_INIT__MSG__DETAIL__POSE6_D__BUILDER_HPP_

#include "lidar_imu_init/msg/detail/pose6_d__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace lidar_imu_init
{

namespace msg
{

namespace builder
{

class Init_Pose6D_rot
{
public:
  explicit Init_Pose6D_rot(::lidar_imu_init::msg::Pose6D & msg)
  : msg_(msg)
  {}
  ::lidar_imu_init::msg::Pose6D rot(::lidar_imu_init::msg::Pose6D::_rot_type arg)
  {
    msg_.rot = std::move(arg);
    return std::move(msg_);
  }

private:
  ::lidar_imu_init::msg::Pose6D msg_;
};

class Init_Pose6D_pos
{
public:
  explicit Init_Pose6D_pos(::lidar_imu_init::msg::Pose6D & msg)
  : msg_(msg)
  {}
  Init_Pose6D_rot pos(::lidar_imu_init::msg::Pose6D::_pos_type arg)
  {
    msg_.pos = std::move(arg);
    return Init_Pose6D_rot(msg_);
  }

private:
  ::lidar_imu_init::msg::Pose6D msg_;
};

class Init_Pose6D_vel
{
public:
  explicit Init_Pose6D_vel(::lidar_imu_init::msg::Pose6D & msg)
  : msg_(msg)
  {}
  Init_Pose6D_pos vel(::lidar_imu_init::msg::Pose6D::_vel_type arg)
  {
    msg_.vel = std::move(arg);
    return Init_Pose6D_pos(msg_);
  }

private:
  ::lidar_imu_init::msg::Pose6D msg_;
};

class Init_Pose6D_gyr
{
public:
  explicit Init_Pose6D_gyr(::lidar_imu_init::msg::Pose6D & msg)
  : msg_(msg)
  {}
  Init_Pose6D_vel gyr(::lidar_imu_init::msg::Pose6D::_gyr_type arg)
  {
    msg_.gyr = std::move(arg);
    return Init_Pose6D_vel(msg_);
  }

private:
  ::lidar_imu_init::msg::Pose6D msg_;
};

class Init_Pose6D_acc
{
public:
  explicit Init_Pose6D_acc(::lidar_imu_init::msg::Pose6D & msg)
  : msg_(msg)
  {}
  Init_Pose6D_gyr acc(::lidar_imu_init::msg::Pose6D::_acc_type arg)
  {
    msg_.acc = std::move(arg);
    return Init_Pose6D_gyr(msg_);
  }

private:
  ::lidar_imu_init::msg::Pose6D msg_;
};

class Init_Pose6D_offset_time
{
public:
  Init_Pose6D_offset_time()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Pose6D_acc offset_time(::lidar_imu_init::msg::Pose6D::_offset_time_type arg)
  {
    msg_.offset_time = std::move(arg);
    return Init_Pose6D_acc(msg_);
  }

private:
  ::lidar_imu_init::msg::Pose6D msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::lidar_imu_init::msg::Pose6D>()
{
  return lidar_imu_init::msg::builder::Init_Pose6D_offset_time();
}

}  // namespace lidar_imu_init

#endif  // LIDAR_IMU_INIT__MSG__DETAIL__POSE6_D__BUILDER_HPP_
