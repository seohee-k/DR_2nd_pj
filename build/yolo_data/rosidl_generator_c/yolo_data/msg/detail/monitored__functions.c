// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from yolo_data:msg/Monitored.idl
// generated code does not contain a copyright notice
#include "yolo_data/msg/detail/monitored__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `class_name`
#include "rosidl_runtime_c/string_functions.h"

bool
yolo_data__msg__Monitored__init(yolo_data__msg__Monitored * msg)
{
  if (!msg) {
    return false;
  }
  // class_name
  if (!rosidl_runtime_c__String__init(&msg->class_name)) {
    yolo_data__msg__Monitored__fini(msg);
    return false;
  }
  // confidence
  return true;
}

void
yolo_data__msg__Monitored__fini(yolo_data__msg__Monitored * msg)
{
  if (!msg) {
    return;
  }
  // class_name
  rosidl_runtime_c__String__fini(&msg->class_name);
  // confidence
}

bool
yolo_data__msg__Monitored__are_equal(const yolo_data__msg__Monitored * lhs, const yolo_data__msg__Monitored * rhs)
{
  if (!lhs || !rhs) {
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
  return true;
}

bool
yolo_data__msg__Monitored__copy(
  const yolo_data__msg__Monitored * input,
  yolo_data__msg__Monitored * output)
{
  if (!input || !output) {
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
  return true;
}

yolo_data__msg__Monitored *
yolo_data__msg__Monitored__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  yolo_data__msg__Monitored * msg = (yolo_data__msg__Monitored *)allocator.allocate(sizeof(yolo_data__msg__Monitored), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(yolo_data__msg__Monitored));
  bool success = yolo_data__msg__Monitored__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
yolo_data__msg__Monitored__destroy(yolo_data__msg__Monitored * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    yolo_data__msg__Monitored__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
yolo_data__msg__Monitored__Sequence__init(yolo_data__msg__Monitored__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  yolo_data__msg__Monitored * data = NULL;

  if (size) {
    data = (yolo_data__msg__Monitored *)allocator.zero_allocate(size, sizeof(yolo_data__msg__Monitored), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = yolo_data__msg__Monitored__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        yolo_data__msg__Monitored__fini(&data[i - 1]);
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
yolo_data__msg__Monitored__Sequence__fini(yolo_data__msg__Monitored__Sequence * array)
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
      yolo_data__msg__Monitored__fini(&array->data[i]);
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

yolo_data__msg__Monitored__Sequence *
yolo_data__msg__Monitored__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  yolo_data__msg__Monitored__Sequence * array = (yolo_data__msg__Monitored__Sequence *)allocator.allocate(sizeof(yolo_data__msg__Monitored__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = yolo_data__msg__Monitored__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
yolo_data__msg__Monitored__Sequence__destroy(yolo_data__msg__Monitored__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    yolo_data__msg__Monitored__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
yolo_data__msg__Monitored__Sequence__are_equal(const yolo_data__msg__Monitored__Sequence * lhs, const yolo_data__msg__Monitored__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!yolo_data__msg__Monitored__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
yolo_data__msg__Monitored__Sequence__copy(
  const yolo_data__msg__Monitored__Sequence * input,
  yolo_data__msg__Monitored__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(yolo_data__msg__Monitored);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    yolo_data__msg__Monitored * data =
      (yolo_data__msg__Monitored *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!yolo_data__msg__Monitored__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          yolo_data__msg__Monitored__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!yolo_data__msg__Monitored__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
