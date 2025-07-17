// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from yoloinference:msg/BoundingBoxes.idl
// generated code does not contain a copyright notice

#ifndef YOLOINFERENCE__MSG__DETAIL__BOUNDING_BOXES__STRUCT_H_
#define YOLOINFERENCE__MSG__DETAIL__BOUNDING_BOXES__STRUCT_H_

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
// Member 'boxes'
#include "yoloinference/msg/detail/bounding_box__struct.h"

/// Struct defined in msg/BoundingBoxes in the package yoloinference.
typedef struct yoloinference__msg__BoundingBoxes
{
  std_msgs__msg__Header header;
  yoloinference__msg__BoundingBox__Sequence boxes;
} yoloinference__msg__BoundingBoxes;

// Struct for a sequence of yoloinference__msg__BoundingBoxes.
typedef struct yoloinference__msg__BoundingBoxes__Sequence
{
  yoloinference__msg__BoundingBoxes * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} yoloinference__msg__BoundingBoxes__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // YOLOINFERENCE__MSG__DETAIL__BOUNDING_BOXES__STRUCT_H_
