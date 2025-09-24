// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from lidar_imu_init:msg/States.idl
// generated code does not contain a copyright notice
#include "lidar_imu_init/msg/detail/states__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "lidar_imu_init/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "lidar_imu_init/msg/detail/states__struct.h"
#include "lidar_imu_init/msg/detail/states__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "rosidl_runtime_c/primitives_sequence.h"  // bias_acc, bias_gyr, cov, gravity, pos_end, rot_end, vel_end
#include "rosidl_runtime_c/primitives_sequence_functions.h"  // bias_acc, bias_gyr, cov, gravity, pos_end, rot_end, vel_end
#include "std_msgs/msg/detail/header__functions.h"  // header

// forward declare type support functions
ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_lidar_imu_init
size_t get_serialized_size_std_msgs__msg__Header(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_lidar_imu_init
size_t max_serialized_size_std_msgs__msg__Header(
  bool & full_bounded,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_lidar_imu_init
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, std_msgs, msg, Header)();


using _States__ros_msg_type = lidar_imu_init__msg__States;

static bool _States__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _States__ros_msg_type * ros_message = static_cast<const _States__ros_msg_type *>(untyped_ros_message);
  // Field name: header
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, std_msgs, msg, Header
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->header, cdr))
    {
      return false;
    }
  }

  // Field name: rot_end
  {
    size_t size = ros_message->rot_end.size;
    auto array_ptr = ros_message->rot_end.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: pos_end
  {
    size_t size = ros_message->pos_end.size;
    auto array_ptr = ros_message->pos_end.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: vel_end
  {
    size_t size = ros_message->vel_end.size;
    auto array_ptr = ros_message->vel_end.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: bias_gyr
  {
    size_t size = ros_message->bias_gyr.size;
    auto array_ptr = ros_message->bias_gyr.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: bias_acc
  {
    size_t size = ros_message->bias_acc.size;
    auto array_ptr = ros_message->bias_acc.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: gravity
  {
    size_t size = ros_message->gravity.size;
    auto array_ptr = ros_message->gravity.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: cov
  {
    size_t size = ros_message->cov.size;
    auto array_ptr = ros_message->cov.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  return true;
}

static bool _States__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _States__ros_msg_type * ros_message = static_cast<_States__ros_msg_type *>(untyped_ros_message);
  // Field name: header
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, std_msgs, msg, Header
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->header))
    {
      return false;
    }
  }

  // Field name: rot_end
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->rot_end.data) {
      rosidl_runtime_c__double__Sequence__fini(&ros_message->rot_end);
    }
    if (!rosidl_runtime_c__double__Sequence__init(&ros_message->rot_end, size)) {
      return "failed to create array for field 'rot_end'";
    }
    auto array_ptr = ros_message->rot_end.data;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: pos_end
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->pos_end.data) {
      rosidl_runtime_c__double__Sequence__fini(&ros_message->pos_end);
    }
    if (!rosidl_runtime_c__double__Sequence__init(&ros_message->pos_end, size)) {
      return "failed to create array for field 'pos_end'";
    }
    auto array_ptr = ros_message->pos_end.data;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: vel_end
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->vel_end.data) {
      rosidl_runtime_c__double__Sequence__fini(&ros_message->vel_end);
    }
    if (!rosidl_runtime_c__double__Sequence__init(&ros_message->vel_end, size)) {
      return "failed to create array for field 'vel_end'";
    }
    auto array_ptr = ros_message->vel_end.data;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: bias_gyr
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->bias_gyr.data) {
      rosidl_runtime_c__double__Sequence__fini(&ros_message->bias_gyr);
    }
    if (!rosidl_runtime_c__double__Sequence__init(&ros_message->bias_gyr, size)) {
      return "failed to create array for field 'bias_gyr'";
    }
    auto array_ptr = ros_message->bias_gyr.data;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: bias_acc
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->bias_acc.data) {
      rosidl_runtime_c__double__Sequence__fini(&ros_message->bias_acc);
    }
    if (!rosidl_runtime_c__double__Sequence__init(&ros_message->bias_acc, size)) {
      return "failed to create array for field 'bias_acc'";
    }
    auto array_ptr = ros_message->bias_acc.data;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: gravity
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->gravity.data) {
      rosidl_runtime_c__double__Sequence__fini(&ros_message->gravity);
    }
    if (!rosidl_runtime_c__double__Sequence__init(&ros_message->gravity, size)) {
      return "failed to create array for field 'gravity'";
    }
    auto array_ptr = ros_message->gravity.data;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: cov
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->cov.data) {
      rosidl_runtime_c__double__Sequence__fini(&ros_message->cov);
    }
    if (!rosidl_runtime_c__double__Sequence__init(&ros_message->cov, size)) {
      return "failed to create array for field 'cov'";
    }
    auto array_ptr = ros_message->cov.data;
    cdr.deserializeArray(array_ptr, size);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_lidar_imu_init
size_t get_serialized_size_lidar_imu_init__msg__States(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _States__ros_msg_type * ros_message = static_cast<const _States__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name header

  current_alignment += get_serialized_size_std_msgs__msg__Header(
    &(ros_message->header), current_alignment);
  // field.name rot_end
  {
    size_t array_size = ros_message->rot_end.size;
    auto array_ptr = ros_message->rot_end.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name pos_end
  {
    size_t array_size = ros_message->pos_end.size;
    auto array_ptr = ros_message->pos_end.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name vel_end
  {
    size_t array_size = ros_message->vel_end.size;
    auto array_ptr = ros_message->vel_end.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name bias_gyr
  {
    size_t array_size = ros_message->bias_gyr.size;
    auto array_ptr = ros_message->bias_gyr.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name bias_acc
  {
    size_t array_size = ros_message->bias_acc.size;
    auto array_ptr = ros_message->bias_acc.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name gravity
  {
    size_t array_size = ros_message->gravity.size;
    auto array_ptr = ros_message->gravity.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name cov
  {
    size_t array_size = ros_message->cov.size;
    auto array_ptr = ros_message->cov.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _States__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_lidar_imu_init__msg__States(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_lidar_imu_init
size_t max_serialized_size_lidar_imu_init__msg__States(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

  // member: header
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        max_serialized_size_std_msgs__msg__Header(
        full_bounded, current_alignment);
    }
  }
  // member: rot_end
  {
    size_t array_size = 0;
    full_bounded = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: pos_end
  {
    size_t array_size = 0;
    full_bounded = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: vel_end
  {
    size_t array_size = 0;
    full_bounded = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: bias_gyr
  {
    size_t array_size = 0;
    full_bounded = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: bias_acc
  {
    size_t array_size = 0;
    full_bounded = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: gravity
  {
    size_t array_size = 0;
    full_bounded = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: cov
  {
    size_t array_size = 0;
    full_bounded = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  return current_alignment - initial_alignment;
}

static size_t _States__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_lidar_imu_init__msg__States(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_States = {
  "lidar_imu_init::msg",
  "States",
  _States__cdr_serialize,
  _States__cdr_deserialize,
  _States__get_serialized_size,
  _States__max_serialized_size
};

static rosidl_message_type_support_t _States__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_States,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, lidar_imu_init, msg, States)() {
  return &_States__type_support;
}

#if defined(__cplusplus)
}
#endif
