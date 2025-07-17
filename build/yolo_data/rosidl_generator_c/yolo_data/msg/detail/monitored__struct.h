// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from yolo_data:msg/Monitored.idl
// generated code does not contain a copyright notice

#ifndef YOLO_DATA__MSG__DETAIL__MONITORED__STRUCT_H_
#define YOLO_DATA__MSG__DETAIL__MONITORED__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'class_name'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/Monitored in the package yolo_data.
typedef struct yolo_data__msg__Monitored
{
  rosidl_runtime_c__String class_name;
  float confidence;
} yolo_data__msg__Monitored;

// Struct for a sequence of yolo_data__msg__Monitored.
typedef struct yolo_data__msg__Monitored__Sequence
{
  yolo_data__msg__Monitored * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} yolo_data__msg__Monitored__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // YOLO_DATA__MSG__DETAIL__MONITORED__STRUCT_H_
