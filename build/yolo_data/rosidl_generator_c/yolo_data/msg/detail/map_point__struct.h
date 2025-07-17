// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from yolo_data:msg/MapPoint.idl
// generated code does not contain a copyright notice

#ifndef YOLO_DATA__MSG__DETAIL__MAP_POINT__STRUCT_H_
#define YOLO_DATA__MSG__DETAIL__MAP_POINT__STRUCT_H_

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

/// Struct defined in msg/MapPoint in the package yolo_data.
typedef struct yolo_data__msg__MapPoint
{
  std_msgs__msg__Header header;
  float x;
  float y;
} yolo_data__msg__MapPoint;

// Struct for a sequence of yolo_data__msg__MapPoint.
typedef struct yolo_data__msg__MapPoint__Sequence
{
  yolo_data__msg__MapPoint * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} yolo_data__msg__MapPoint__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // YOLO_DATA__MSG__DETAIL__MAP_POINT__STRUCT_H_
