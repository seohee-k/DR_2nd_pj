// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from yolo_data:msg/MonitorData.idl
// generated code does not contain a copyright notice

#ifndef YOLO_DATA__MSG__DETAIL__MONITOR_DATA__STRUCT_H_
#define YOLO_DATA__MSG__DETAIL__MONITOR_DATA__STRUCT_H_

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
#include "yolo_data/msg/detail/monitored__struct.h"
// Member 'points'
#include "yolo_data/msg/detail/map_point__struct.h"

/// Struct defined in msg/MonitorData in the package yolo_data.
typedef struct yolo_data__msg__MonitorData
{
  std_msgs__msg__Header header;
  yolo_data__msg__Monitored__Sequence boxes;
  yolo_data__msg__MapPoint__Sequence points;
} yolo_data__msg__MonitorData;

// Struct for a sequence of yolo_data__msg__MonitorData.
typedef struct yolo_data__msg__MonitorData__Sequence
{
  yolo_data__msg__MonitorData * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} yolo_data__msg__MonitorData__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // YOLO_DATA__MSG__DETAIL__MONITOR_DATA__STRUCT_H_
