// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from lidar_imu_init:msg/Pose6D.idl
// generated code does not contain a copyright notice
#include "lidar_imu_init/msg/detail/pose6_d__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
lidar_imu_init__msg__Pose6D__init(lidar_imu_init__msg__Pose6D * msg)
{
  if (!msg) {
    return false;
  }
  // offset_time
  // acc
  // gyr
  // vel
  // pos
  // rot
  return true;
}

void
lidar_imu_init__msg__Pose6D__fini(lidar_imu_init__msg__Pose6D * msg)
{
  if (!msg) {
    return;
  }
  // offset_time
  // acc
  // gyr
  // vel
  // pos
  // rot
}

bool
lidar_imu_init__msg__Pose6D__are_equal(const lidar_imu_init__msg__Pose6D * lhs, const lidar_imu_init__msg__Pose6D * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // offset_time
  if (lhs->offset_time != rhs->offset_time) {
    return false;
  }
  // acc
  for (size_t i = 0; i < 3; ++i) {
    if (lhs->acc[i] != rhs->acc[i]) {
      return false;
    }
  }
  // gyr
  for (size_t i = 0; i < 3; ++i) {
    if (lhs->gyr[i] != rhs->gyr[i]) {
      return false;
    }
  }
  // vel
  for (size_t i = 0; i < 3; ++i) {
    if (lhs->vel[i] != rhs->vel[i]) {
      return false;
    }
  }
  // pos
  for (size_t i = 0; i < 3; ++i) {
    if (lhs->pos[i] != rhs->pos[i]) {
      return false;
    }
  }
  // rot
  for (size_t i = 0; i < 9; ++i) {
    if (lhs->rot[i] != rhs->rot[i]) {
      return false;
    }
  }
  return true;
}

bool
lidar_imu_init__msg__Pose6D__copy(
  const lidar_imu_init__msg__Pose6D * input,
  lidar_imu_init__msg__Pose6D * output)
{
  if (!input || !output) {
    return false;
  }
  // offset_time
  output->offset_time = input->offset_time;
  // acc
  for (size_t i = 0; i < 3; ++i) {
    output->acc[i] = input->acc[i];
  }
  // gyr
  for (size_t i = 0; i < 3; ++i) {
    output->gyr[i] = input->gyr[i];
  }
  // vel
  for (size_t i = 0; i < 3; ++i) {
    output->vel[i] = input->vel[i];
  }
  // pos
  for (size_t i = 0; i < 3; ++i) {
    output->pos[i] = input->pos[i];
  }
  // rot
  for (size_t i = 0; i < 9; ++i) {
    output->rot[i] = input->rot[i];
  }
  return true;
}

lidar_imu_init__msg__Pose6D *
lidar_imu_init__msg__Pose6D__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  lidar_imu_init__msg__Pose6D * msg = (lidar_imu_init__msg__Pose6D *)allocator.allocate(sizeof(lidar_imu_init__msg__Pose6D), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(lidar_imu_init__msg__Pose6D));
  bool success = lidar_imu_init__msg__Pose6D__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
lidar_imu_init__msg__Pose6D__destroy(lidar_imu_init__msg__Pose6D * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    lidar_imu_init__msg__Pose6D__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
lidar_imu_init__msg__Pose6D__Sequence__init(lidar_imu_init__msg__Pose6D__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  lidar_imu_init__msg__Pose6D * data = NULL;

  if (size) {
    data = (lidar_imu_init__msg__Pose6D *)allocator.zero_allocate(size, sizeof(lidar_imu_init__msg__Pose6D), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = lidar_imu_init__msg__Pose6D__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        lidar_imu_init__msg__Pose6D__fini(&data[i - 1]);
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
lidar_imu_init__msg__Pose6D__Sequence__fini(lidar_imu_init__msg__Pose6D__Sequence * array)
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
      lidar_imu_init__msg__Pose6D__fini(&array->data[i]);
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

lidar_imu_init__msg__Pose6D__Sequence *
lidar_imu_init__msg__Pose6D__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  lidar_imu_init__msg__Pose6D__Sequence * array = (lidar_imu_init__msg__Pose6D__Sequence *)allocator.allocate(sizeof(lidar_imu_init__msg__Pose6D__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = lidar_imu_init__msg__Pose6D__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
lidar_imu_init__msg__Pose6D__Sequence__destroy(lidar_imu_init__msg__Pose6D__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    lidar_imu_init__msg__Pose6D__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
lidar_imu_init__msg__Pose6D__Sequence__are_equal(const lidar_imu_init__msg__Pose6D__Sequence * lhs, const lidar_imu_init__msg__Pose6D__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!lidar_imu_init__msg__Pose6D__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
lidar_imu_init__msg__Pose6D__Sequence__copy(
  const lidar_imu_init__msg__Pose6D__Sequence * input,
  lidar_imu_init__msg__Pose6D__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(lidar_imu_init__msg__Pose6D);
    lidar_imu_init__msg__Pose6D * data =
      (lidar_imu_init__msg__Pose6D *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!lidar_imu_init__msg__Pose6D__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          lidar_imu_init__msg__Pose6D__fini(&data[i]);
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
    if (!lidar_imu_init__msg__Pose6D__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
