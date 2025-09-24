// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from lidar_imu_init:msg/States.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "lidar_imu_init/msg/detail/states__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace lidar_imu_init
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void States_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) lidar_imu_init::msg::States(_init);
}

void States_fini_function(void * message_memory)
{
  auto typed_message = static_cast<lidar_imu_init::msg::States *>(message_memory);
  typed_message->~States();
}

size_t size_function__States__rot_end(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<double> *>(untyped_member);
  return member->size();
}

const void * get_const_function__States__rot_end(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<double> *>(untyped_member);
  return &member[index];
}

void * get_function__States__rot_end(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<double> *>(untyped_member);
  return &member[index];
}

void resize_function__States__rot_end(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<double> *>(untyped_member);
  member->resize(size);
}

size_t size_function__States__pos_end(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<double> *>(untyped_member);
  return member->size();
}

const void * get_const_function__States__pos_end(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<double> *>(untyped_member);
  return &member[index];
}

void * get_function__States__pos_end(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<double> *>(untyped_member);
  return &member[index];
}

void resize_function__States__pos_end(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<double> *>(untyped_member);
  member->resize(size);
}

size_t size_function__States__vel_end(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<double> *>(untyped_member);
  return member->size();
}

const void * get_const_function__States__vel_end(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<double> *>(untyped_member);
  return &member[index];
}

void * get_function__States__vel_end(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<double> *>(untyped_member);
  return &member[index];
}

void resize_function__States__vel_end(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<double> *>(untyped_member);
  member->resize(size);
}

size_t size_function__States__bias_gyr(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<double> *>(untyped_member);
  return member->size();
}

const void * get_const_function__States__bias_gyr(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<double> *>(untyped_member);
  return &member[index];
}

void * get_function__States__bias_gyr(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<double> *>(untyped_member);
  return &member[index];
}

void resize_function__States__bias_gyr(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<double> *>(untyped_member);
  member->resize(size);
}

size_t size_function__States__bias_acc(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<double> *>(untyped_member);
  return member->size();
}

const void * get_const_function__States__bias_acc(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<double> *>(untyped_member);
  return &member[index];
}

void * get_function__States__bias_acc(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<double> *>(untyped_member);
  return &member[index];
}

void resize_function__States__bias_acc(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<double> *>(untyped_member);
  member->resize(size);
}

size_t size_function__States__gravity(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<double> *>(untyped_member);
  return member->size();
}

const void * get_const_function__States__gravity(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<double> *>(untyped_member);
  return &member[index];
}

void * get_function__States__gravity(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<double> *>(untyped_member);
  return &member[index];
}

void resize_function__States__gravity(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<double> *>(untyped_member);
  member->resize(size);
}

size_t size_function__States__cov(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<double> *>(untyped_member);
  return member->size();
}

const void * get_const_function__States__cov(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<double> *>(untyped_member);
  return &member[index];
}

void * get_function__States__cov(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<double> *>(untyped_member);
  return &member[index];
}

void resize_function__States__cov(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<double> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember States_message_member_array[8] = {
  {
    "header",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Header>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(lidar_imu_init::msg::States, header),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "rot_end",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(lidar_imu_init::msg::States, rot_end),  // bytes offset in struct
    nullptr,  // default value
    size_function__States__rot_end,  // size() function pointer
    get_const_function__States__rot_end,  // get_const(index) function pointer
    get_function__States__rot_end,  // get(index) function pointer
    resize_function__States__rot_end  // resize(index) function pointer
  },
  {
    "pos_end",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(lidar_imu_init::msg::States, pos_end),  // bytes offset in struct
    nullptr,  // default value
    size_function__States__pos_end,  // size() function pointer
    get_const_function__States__pos_end,  // get_const(index) function pointer
    get_function__States__pos_end,  // get(index) function pointer
    resize_function__States__pos_end  // resize(index) function pointer
  },
  {
    "vel_end",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(lidar_imu_init::msg::States, vel_end),  // bytes offset in struct
    nullptr,  // default value
    size_function__States__vel_end,  // size() function pointer
    get_const_function__States__vel_end,  // get_const(index) function pointer
    get_function__States__vel_end,  // get(index) function pointer
    resize_function__States__vel_end  // resize(index) function pointer
  },
  {
    "bias_gyr",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(lidar_imu_init::msg::States, bias_gyr),  // bytes offset in struct
    nullptr,  // default value
    size_function__States__bias_gyr,  // size() function pointer
    get_const_function__States__bias_gyr,  // get_const(index) function pointer
    get_function__States__bias_gyr,  // get(index) function pointer
    resize_function__States__bias_gyr  // resize(index) function pointer
  },
  {
    "bias_acc",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(lidar_imu_init::msg::States, bias_acc),  // bytes offset in struct
    nullptr,  // default value
    size_function__States__bias_acc,  // size() function pointer
    get_const_function__States__bias_acc,  // get_const(index) function pointer
    get_function__States__bias_acc,  // get(index) function pointer
    resize_function__States__bias_acc  // resize(index) function pointer
  },
  {
    "gravity",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(lidar_imu_init::msg::States, gravity),  // bytes offset in struct
    nullptr,  // default value
    size_function__States__gravity,  // size() function pointer
    get_const_function__States__gravity,  // get_const(index) function pointer
    get_function__States__gravity,  // get(index) function pointer
    resize_function__States__gravity  // resize(index) function pointer
  },
  {
    "cov",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(lidar_imu_init::msg::States, cov),  // bytes offset in struct
    nullptr,  // default value
    size_function__States__cov,  // size() function pointer
    get_const_function__States__cov,  // get_const(index) function pointer
    get_function__States__cov,  // get(index) function pointer
    resize_function__States__cov  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers States_message_members = {
  "lidar_imu_init::msg",  // message namespace
  "States",  // message name
  8,  // number of fields
  sizeof(lidar_imu_init::msg::States),
  States_message_member_array,  // message members
  States_init_function,  // function to initialize message memory (memory has to be allocated)
  States_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t States_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &States_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace lidar_imu_init


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<lidar_imu_init::msg::States>()
{
  return &::lidar_imu_init::msg::rosidl_typesupport_introspection_cpp::States_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, lidar_imu_init, msg, States)() {
  return &::lidar_imu_init::msg::rosidl_typesupport_introspection_cpp::States_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
