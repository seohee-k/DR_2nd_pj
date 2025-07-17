// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from yolo_data:msg/MapPoint.idl
// generated code does not contain a copyright notice
#include "yolo_data/msg/detail/map_point__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
yolo_data__msg__MapPoint__init(yolo_data__msg__MapPoint * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    yolo_data__msg__MapPoint__fini(msg);
    return false;
  }
  // x
  // y
  return true;
}

void
yolo_data__msg__MapPoint__fini(yolo_data__msg__MapPoint * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // x
  // y
}

bool
yolo_data__msg__MapPoint__are_equal(const yolo_data__msg__MapPoint * lhs, const yolo_data__msg__MapPoint * rhs)
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
  // x
  if (lhs->x != rhs->x) {
    return false;
  }
  // y
  if (lhs->y != rhs->y) {
    return false;
  }
  return true;
}

bool
yolo_data__msg__MapPoint__copy(
  const yolo_data__msg__MapPoint * input,
  yolo_data__msg__MapPoint * output)
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
  // x
  output->x = input->x;
  // y
  output->y = input->y;
  return true;
}

yolo_data__msg__MapPoint *
yolo_data__msg__MapPoint__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  yolo_data__msg__MapPoint * msg = (yolo_data__msg__MapPoint *)allocator.allocate(sizeof(yolo_data__msg__MapPoint), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(yolo_data__msg__MapPoint));
  bool success = yolo_data__msg__MapPoint__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
yolo_data__msg__MapPoint__destroy(yolo_data__msg__MapPoint * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    yolo_data__msg__MapPoint__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
yolo_data__msg__MapPoint__Sequence__init(yolo_data__msg__MapPoint__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  yolo_data__msg__MapPoint * data = NULL;

  if (size) {
    data = (yolo_data__msg__MapPoint *)allocator.zero_allocate(size, sizeof(yolo_data__msg__MapPoint), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = yolo_data__msg__MapPoint__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        yolo_data__msg__MapPoint__fini(&data[i - 1]);
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
yolo_data__msg__MapPoint__Sequence__fini(yolo_data__msg__MapPoint__Sequence * array)
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
      yolo_data__msg__MapPoint__fini(&array->data[i]);
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

yolo_data__msg__MapPoint__Sequence *
yolo_data__msg__MapPoint__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  yolo_data__msg__MapPoint__Sequence * array = (yolo_data__msg__MapPoint__Sequence *)allocator.allocate(sizeof(yolo_data__msg__MapPoint__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = yolo_data__msg__MapPoint__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
yolo_data__msg__MapPoint__Sequence__destroy(yolo_data__msg__MapPoint__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    yolo_data__msg__MapPoint__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
yolo_data__msg__MapPoint__Sequence__are_equal(const yolo_data__msg__MapPoint__Sequence * lhs, const yolo_data__msg__MapPoint__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!yolo_data__msg__MapPoint__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
yolo_data__msg__MapPoint__Sequence__copy(
  const yolo_data__msg__MapPoint__Sequence * input,
  yolo_data__msg__MapPoint__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(yolo_data__msg__MapPoint);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    yolo_data__msg__MapPoint * data =
      (yolo_data__msg__MapPoint *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!yolo_data__msg__MapPoint__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          yolo_data__msg__MapPoint__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!yolo_data__msg__MapPoint__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
