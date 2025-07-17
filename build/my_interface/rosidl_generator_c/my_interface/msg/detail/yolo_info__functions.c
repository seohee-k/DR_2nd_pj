// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from my_interface:msg/YoloInfo.idl
// generated code does not contain a copyright notice
#include "my_interface/msg/detail/yolo_info__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `class_name`
#include "rosidl_runtime_c/string_functions.h"

bool
my_interface__msg__YoloInfo__init(my_interface__msg__YoloInfo * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    my_interface__msg__YoloInfo__fini(msg);
    return false;
  }
  // class_name
  if (!rosidl_runtime_c__String__init(&msg->class_name)) {
    my_interface__msg__YoloInfo__fini(msg);
    return false;
  }
  // confidence
  // x
  // y
  return true;
}

void
my_interface__msg__YoloInfo__fini(my_interface__msg__YoloInfo * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // class_name
  rosidl_runtime_c__String__fini(&msg->class_name);
  // confidence
  // x
  // y
}

bool
my_interface__msg__YoloInfo__are_equal(const my_interface__msg__YoloInfo * lhs, const my_interface__msg__YoloInfo * rhs)
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
  // class_name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->class_name), &(rhs->class_name)))
  {
    return false;
  }
  // confidence
  if (lhs->confidence != rhs->confidence) {
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
my_interface__msg__YoloInfo__copy(
  const my_interface__msg__YoloInfo * input,
  my_interface__msg__YoloInfo * output)
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
  // class_name
  if (!rosidl_runtime_c__String__copy(
      &(input->class_name), &(output->class_name)))
  {
    return false;
  }
  // confidence
  output->confidence = input->confidence;
  // x
  output->x = input->x;
  // y
  output->y = input->y;
  return true;
}

my_interface__msg__YoloInfo *
my_interface__msg__YoloInfo__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  my_interface__msg__YoloInfo * msg = (my_interface__msg__YoloInfo *)allocator.allocate(sizeof(my_interface__msg__YoloInfo), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(my_interface__msg__YoloInfo));
  bool success = my_interface__msg__YoloInfo__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
my_interface__msg__YoloInfo__destroy(my_interface__msg__YoloInfo * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    my_interface__msg__YoloInfo__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
my_interface__msg__YoloInfo__Sequence__init(my_interface__msg__YoloInfo__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  my_interface__msg__YoloInfo * data = NULL;

  if (size) {
    data = (my_interface__msg__YoloInfo *)allocator.zero_allocate(size, sizeof(my_interface__msg__YoloInfo), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = my_interface__msg__YoloInfo__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        my_interface__msg__YoloInfo__fini(&data[i - 1]);
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
my_interface__msg__YoloInfo__Sequence__fini(my_interface__msg__YoloInfo__Sequence * array)
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
      my_interface__msg__YoloInfo__fini(&array->data[i]);
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

my_interface__msg__YoloInfo__Sequence *
my_interface__msg__YoloInfo__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  my_interface__msg__YoloInfo__Sequence * array = (my_interface__msg__YoloInfo__Sequence *)allocator.allocate(sizeof(my_interface__msg__YoloInfo__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = my_interface__msg__YoloInfo__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
my_interface__msg__YoloInfo__Sequence__destroy(my_interface__msg__YoloInfo__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    my_interface__msg__YoloInfo__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
my_interface__msg__YoloInfo__Sequence__are_equal(const my_interface__msg__YoloInfo__Sequence * lhs, const my_interface__msg__YoloInfo__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!my_interface__msg__YoloInfo__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
my_interface__msg__YoloInfo__Sequence__copy(
  const my_interface__msg__YoloInfo__Sequence * input,
  my_interface__msg__YoloInfo__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(my_interface__msg__YoloInfo);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    my_interface__msg__YoloInfo * data =
      (my_interface__msg__YoloInfo *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!my_interface__msg__YoloInfo__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          my_interface__msg__YoloInfo__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!my_interface__msg__YoloInfo__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
