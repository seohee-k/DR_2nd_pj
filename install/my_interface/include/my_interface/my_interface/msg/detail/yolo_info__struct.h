// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from my_interface:msg/YoloInfo.idl
// generated code does not contain a copyright notice

#ifndef MY_INTERFACE__MSG__DETAIL__YOLO_INFO__STRUCT_H_
#define MY_INTERFACE__MSG__DETAIL__YOLO_INFO__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'class_name'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/YoloInfo in the package my_interface.
typedef struct my_interface__msg__YoloInfo
{
  std_msgs__msg__Header header;
  rosidl_runtime_c__String class_name;
  double confidence;
  double x;
  double y;
} my_interface__msg__YoloInfo;

// Struct for a sequence of my_interface__msg__YoloInfo.
typedef struct my_interface__msg__YoloInfo__Sequence
{
  my_interface__msg__YoloInfo * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} my_interface__msg__YoloInfo__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MY_INTERFACE__MSG__DETAIL__YOLO_INFO__STRUCT_H_
