// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from yoloinference:msg/BoundingBox.idl
// generated code does not contain a copyright notice

#ifndef YOLOINFERENCE__MSG__DETAIL__BOUNDING_BOX__STRUCT_H_
#define YOLOINFERENCE__MSG__DETAIL__BOUNDING_BOX__STRUCT_H_

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

/// Struct defined in msg/BoundingBox in the package yoloinference.
typedef struct yoloinference__msg__BoundingBox
{
  rosidl_runtime_c__String class_name;
  float confidence;
  float x1;
  float y1;
  float x2;
  float y2;
} yoloinference__msg__BoundingBox;

// Struct for a sequence of yoloinference__msg__BoundingBox.
typedef struct yoloinference__msg__BoundingBox__Sequence
{
  yoloinference__msg__BoundingBox * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} yoloinference__msg__BoundingBox__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // YOLOINFERENCE__MSG__DETAIL__BOUNDING_BOX__STRUCT_H_
