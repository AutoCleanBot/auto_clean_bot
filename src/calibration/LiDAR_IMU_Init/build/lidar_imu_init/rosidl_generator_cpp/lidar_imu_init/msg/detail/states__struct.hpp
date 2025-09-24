// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from lidar_imu_init:msg/States.idl
// generated code does not contain a copyright notice

#ifndef LIDAR_IMU_INIT__MSG__DETAIL__STATES__STRUCT_HPP_
#define LIDAR_IMU_INIT__MSG__DETAIL__STATES__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__lidar_imu_init__msg__States __attribute__((deprecated))
#else
# define DEPRECATED__lidar_imu_init__msg__States __declspec(deprecated)
#endif

namespace lidar_imu_init
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct States_
{
  using Type = States_<ContainerAllocator>;

  explicit States_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    (void)_init;
  }

  explicit States_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _rot_end_type =
    std::vector<double, typename ContainerAllocator::template rebind<double>::other>;
  _rot_end_type rot_end;
  using _pos_end_type =
    std::vector<double, typename ContainerAllocator::template rebind<double>::other>;
  _pos_end_type pos_end;
  using _vel_end_type =
    std::vector<double, typename ContainerAllocator::template rebind<double>::other>;
  _vel_end_type vel_end;
  using _bias_gyr_type =
    std::vector<double, typename ContainerAllocator::template rebind<double>::other>;
  _bias_gyr_type bias_gyr;
  using _bias_acc_type =
    std::vector<double, typename ContainerAllocator::template rebind<double>::other>;
  _bias_acc_type bias_acc;
  using _gravity_type =
    std::vector<double, typename ContainerAllocator::template rebind<double>::other>;
  _gravity_type gravity;
  using _cov_type =
    std::vector<double, typename ContainerAllocator::template rebind<double>::other>;
  _cov_type cov;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__rot_end(
    const std::vector<double, typename ContainerAllocator::template rebind<double>::other> & _arg)
  {
    this->rot_end = _arg;
    return *this;
  }
  Type & set__pos_end(
    const std::vector<double, typename ContainerAllocator::template rebind<double>::other> & _arg)
  {
    this->pos_end = _arg;
    return *this;
  }
  Type & set__vel_end(
    const std::vector<double, typename ContainerAllocator::template rebind<double>::other> & _arg)
  {
    this->vel_end = _arg;
    return *this;
  }
  Type & set__bias_gyr(
    const std::vector<double, typename ContainerAllocator::template rebind<double>::other> & _arg)
  {
    this->bias_gyr = _arg;
    return *this;
  }
  Type & set__bias_acc(
    const std::vector<double, typename ContainerAllocator::template rebind<double>::other> & _arg)
  {
    this->bias_acc = _arg;
    return *this;
  }
  Type & set__gravity(
    const std::vector<double, typename ContainerAllocator::template rebind<double>::other> & _arg)
  {
    this->gravity = _arg;
    return *this;
  }
  Type & set__cov(
    const std::vector<double, typename ContainerAllocator::template rebind<double>::other> & _arg)
  {
    this->cov = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    lidar_imu_init::msg::States_<ContainerAllocator> *;
  using ConstRawPtr =
    const lidar_imu_init::msg::States_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<lidar_imu_init::msg::States_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<lidar_imu_init::msg::States_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      lidar_imu_init::msg::States_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<lidar_imu_init::msg::States_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      lidar_imu_init::msg::States_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<lidar_imu_init::msg::States_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<lidar_imu_init::msg::States_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<lidar_imu_init::msg::States_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__lidar_imu_init__msg__States
    std::shared_ptr<lidar_imu_init::msg::States_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__lidar_imu_init__msg__States
    std::shared_ptr<lidar_imu_init::msg::States_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const States_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->rot_end != other.rot_end) {
      return false;
    }
    if (this->pos_end != other.pos_end) {
      return false;
    }
    if (this->vel_end != other.vel_end) {
      return false;
    }
    if (this->bias_gyr != other.bias_gyr) {
      return false;
    }
    if (this->bias_acc != other.bias_acc) {
      return false;
    }
    if (this->gravity != other.gravity) {
      return false;
    }
    if (this->cov != other.cov) {
      return false;
    }
    return true;
  }
  bool operator!=(const States_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct States_

// alias to use template instance with default allocator
using States =
  lidar_imu_init::msg::States_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace lidar_imu_init

#endif  // LIDAR_IMU_INIT__MSG__DETAIL__STATES__STRUCT_HPP_
