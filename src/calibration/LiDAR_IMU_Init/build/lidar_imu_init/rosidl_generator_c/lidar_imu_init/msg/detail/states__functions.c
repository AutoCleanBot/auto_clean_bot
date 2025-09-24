// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from lidar_imu_init:msg/States.idl
// generated code does not contain a copyright notice
#include "lidar_imu_init/msg/detail/states__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `rot_end`
// Member `pos_end`
// Member `vel_end`
// Member `bias_gyr`
// Member `bias_acc`
// Member `gravity`
// Member `cov`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
lidar_imu_init__msg__States__init(lidar_imu_init__msg__States * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    lidar_imu_init__msg__States__fini(msg);
    return false;
  }
  // rot_end
  if (!rosidl_runtime_c__double__Sequence__init(&msg->rot_end, 0)) {
    lidar_imu_init__msg__States__fini(msg);
    return false;
  }
  // pos_end
  if (!rosidl_runtime_c__double__Sequence__init(&msg->pos_end, 0)) {
    lidar_imu_init__msg__States__fini(msg);
    return false;
  }
  // vel_end
  if (!rosidl_runtime_c__double__Sequence__init(&msg->vel_end, 0)) {
    lidar_imu_init__msg__States__fini(msg);
    return false;
  }
  // bias_gyr
  if (!rosidl_runtime_c__double__Sequence__init(&msg->bias_gyr, 0)) {
    lidar_imu_init__msg__States__fini(msg);
    return false;
  }
  // bias_acc
  if (!rosidl_runtime_c__double__Sequence__init(&msg->bias_acc, 0)) {
    lidar_imu_init__msg__States__fini(msg);
    return false;
  }
  // gravity
  if (!rosidl_runtime_c__double__Sequence__init(&msg->gravity, 0)) {
    lidar_imu_init__msg__States__fini(msg);
    return false;
  }
  // cov
  if (!rosidl_runtime_c__double__Sequence__init(&msg->cov, 0)) {
    lidar_imu_init__msg__States__fini(msg);
    return false;
  }
  return true;
}

void
lidar_imu_init__msg__States__fini(lidar_imu_init__msg__States * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // rot_end
  rosidl_runtime_c__double__Sequence__fini(&msg->rot_end);
  // pos_end
  rosidl_runtime_c__double__Sequence__fini(&msg->pos_end);
  // vel_end
  rosidl_runtime_c__double__Sequence__fini(&msg->vel_end);
  // bias_gyr
  rosidl_runtime_c__double__Sequence__fini(&msg->bias_gyr);
  // bias_acc
  rosidl_runtime_c__double__Sequence__fini(&msg->bias_acc);
  // gravity
  rosidl_runtime_c__double__Sequence__fini(&msg->gravity);
  // cov
  rosidl_runtime_c__double__Sequence__fini(&msg->cov);
}

bool
lidar_imu_init__msg__States__are_equal(const lidar_imu_init__msg__States * lhs, const lidar_imu_init__msg__States * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // rot_end
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->rot_end), &(rhs->rot_end)))
  {
    return false;
  }
  // pos_end
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->pos_end), &(rhs->pos_end)))
  {
    return false;
  }
  // vel_end
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->vel_end), &(rhs->vel_end)))
  {
    return false;
  }
  // bias_gyr
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->bias_gyr), &(rhs->bias_gyr)))
  {
    return false;
  }
  // bias_acc
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->bias_acc), &(rhs->bias_acc)))
  {
    return false;
  }
  // gravity
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->gravity), &(rhs->gravity)))
  {
    return false;
  }
  // cov
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->cov), &(rhs->cov)))
  {
    return false;
  }
  return true;
}

bool
lidar_imu_init__msg__States__copy(
  const lidar_imu_init__msg__States * input,
  lidar_imu_init__msg__States * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // rot_end
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->rot_end), &(output->rot_end)))
  {
    return false;
  }
  // pos_end
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->pos_end), &(output->pos_end)))
  {
    return false;
  }
  // vel_end
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->vel_end), &(output->vel_end)))
  {
    return false;
  }
  // bias_gyr
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->bias_gyr), &(output->bias_gyr)))
  {
    return false;
  }
  // bias_acc
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->bias_acc), &(output->bias_acc)))
  {
    return false;
  }
  // gravity
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->gravity), &(output->gravity)))
  {
    return false;
  }
  // cov
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->cov), &(output->cov)))
  {
    return false;
  }
  return true;
}

lidar_imu_init__msg__States *
lidar_imu_init__msg__States__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  lidar_imu_init__msg__States * msg = (lidar_imu_init__msg__States *)allocator.allocate(sizeof(lidar_imu_init__msg__States), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(lidar_imu_init__msg__States));
  bool success = lidar_imu_init__msg__States__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
lidar_imu_init__msg__States__destroy(lidar_imu_init__msg__States * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    lidar_imu_init__msg__States__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
lidar_imu_init__msg__States__Sequence__init(lidar_imu_init__msg__States__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  lidar_imu_init__msg__States * data = NULL;

  if (size) {
    data = (lidar_imu_init__msg__States *)allocator.zero_allocate(size, sizeof(lidar_imu_init__msg__States), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = lidar_imu_init__msg__States__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        lidar_imu_init__msg__States__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
lidar_imu_init__msg__States__Sequence__fini(lidar_imu_init__msg__States__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      lidar_imu_init__msg__States__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

lidar_imu_init__msg__States__Sequence *
lidar_imu_init__msg__States__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  lidar_imu_init__msg__States__Sequence * array = (lidar_imu_init__msg__States__Sequence *)allocator.allocate(sizeof(lidar_imu_init__msg__States__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = lidar_imu_init__msg__States__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
lidar_imu_init__msg__States__Sequence__destroy(lidar_imu_init__msg__States__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    lidar_imu_init__msg__States__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
lidar_imu_init__msg__States__Sequence__are_equal(const lidar_imu_init__msg__States__Sequence * lhs, const lidar_imu_init__msg__States__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!lidar_imu_init__msg__States__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
lidar_imu_init__msg__States__Sequence__copy(
  const lidar_imu_init__msg__States__Sequence * input,
  lidar_imu_init__msg__States__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(lidar_imu_init__msg__States);
    lidar_imu_init__msg__States * data =
      (lidar_imu_init__msg__States *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!lidar_imu_init__msg__States__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          lidar_imu_init__msg__States__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!lidar_imu_init__msg__States__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
