// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from yolo_data:msg/MonitorData.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "yolo_data/msg/detail/monitor_data__rosidl_typesupport_introspection_c.h"
#include "yolo_data/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "yolo_data/msg/detail/monitor_data__functions.h"
#include "yolo_data/msg/detail/monitor_data__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `boxes`
#include "yolo_data/msg/monitored.h"
// Member `boxes`
#include "yolo_data/msg/detail/monitored__rosidl_typesupport_introspection_c.h"
// Member `points`
#include "yolo_data/msg/map_point.h"
// Member `points`
#include "yolo_data/msg/detail/map_point__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void yolo_data__msg__MonitorData__rosidl_typesupport_introspection_c__MonitorData_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  yolo_data__msg__MonitorData__init(message_memory);
}

void yolo_data__msg__MonitorData__rosidl_typesupport_introspection_c__MonitorData_fini_function(void * message_memory)
{
  yolo_data__msg__MonitorData__fini(message_memory);
}

size_t yolo_data__msg__MonitorData__rosidl_typesupport_introspection_c__size_function__MonitorData__boxes(
  const void * untyped_member)
{
  const yolo_data__msg__Monitored__Sequence * member =
    (const yolo_data__msg__Monitored__Sequence *)(untyped_member);
  return member->size;
}

const void * yolo_data__msg__MonitorData__rosidl_typesupport_introspection_c__get_const_function__MonitorData__boxes(
  const void * untyped_member, size_t index)
{
  const yolo_data__msg__Monitored__Sequence * member =
    (const yolo_data__msg__Monitored__Sequence *)(untyped_member);
  return &member->data[index];
}

void * yolo_data__msg__MonitorData__rosidl_typesupport_introspection_c__get_function__MonitorData__boxes(
  void * untyped_member, size_t index)
{
  yolo_data__msg__Monitored__Sequence * member =
    (yolo_data__msg__Monitored__Sequence *)(untyped_member);
  return &member->data[index];
}

void yolo_data__msg__MonitorData__rosidl_typesupport_introspection_c__fetch_function__MonitorData__boxes(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const yolo_data__msg__Monitored * item =
    ((const yolo_data__msg__Monitored *)
    yolo_data__msg__MonitorData__rosidl_typesupport_introspection_c__get_const_function__MonitorData__boxes(untyped_member, index));
  yolo_data__msg__Monitored * value =
    (yolo_data__msg__Monitored *)(untyped_value);
  *value = *item;
}

void yolo_data__msg__MonitorData__rosidl_typesupport_introspection_c__assign_function__MonitorData__boxes(
  void * untyped_member, size_t index, const void * untyped_value)
{
  yolo_data__msg__Monitored * item =
    ((yolo_data__msg__Monitored *)
    yolo_data__msg__MonitorData__rosidl_typesupport_introspection_c__get_function__MonitorData__boxes(untyped_member, index));
  const yolo_data__msg__Monitored * value =
    (const yolo_data__msg__Monitored *)(untyped_value);
  *item = *value;
}

bool yolo_data__msg__MonitorData__rosidl_typesupport_introspection_c__resize_function__MonitorData__boxes(
  void * untyped_member, size_t size)
{
  yolo_data__msg__Monitored__Sequence * member =
    (yolo_data__msg__Monitored__Sequence *)(untyped_member);
  yolo_data__msg__Monitored__Sequence__fini(member);
  return yolo_data__msg__Monitored__Sequence__init(member, size);
}

size_t yolo_data__msg__MonitorData__rosidl_typesupport_introspection_c__size_function__MonitorData__points(
  const void * untyped_member)
{
  const yolo_data__msg__MapPoint__Sequence * member =
    (const yolo_data__msg__MapPoint__Sequence *)(untyped_member);
  return member->size;
}

const void * yolo_data__msg__MonitorData__rosidl_typesupport_introspection_c__get_const_function__MonitorData__points(
  const void * untyped_member, size_t index)
{
  const yolo_data__msg__MapPoint__Sequence * member =
    (const yolo_data__msg__MapPoint__Sequence *)(untyped_member);
  return &member->data[index];
}

void * yolo_data__msg__MonitorData__rosidl_typesupport_introspection_c__get_function__MonitorData__points(
  void * untyped_member, size_t index)
{
  yolo_data__msg__MapPoint__Sequence * member =
    (yolo_data__msg__MapPoint__Sequence *)(untyped_member);
  return &member->data[index];
}

void yolo_data__msg__MonitorData__rosidl_typesupport_introspection_c__fetch_function__MonitorData__points(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const yolo_data__msg__MapPoint * item =
    ((const yolo_data__msg__MapPoint *)
    yolo_data__msg__MonitorData__rosidl_typesupport_introspection_c__get_const_function__MonitorData__points(untyped_member, index));
  yolo_data__msg__MapPoint * value =
    (yolo_data__msg__MapPoint *)(untyped_value);
  *value = *item;
}

void yolo_data__msg__MonitorData__rosidl_typesupport_introspection_c__assign_function__MonitorData__points(
  void * untyped_member, size_t index, const void * untyped_value)
{
  yolo_data__msg__MapPoint * item =
    ((yolo_data__msg__MapPoint *)
    yolo_data__msg__MonitorData__rosidl_typesupport_introspection_c__get_function__MonitorData__points(untyped_member, index));
  const yolo_data__msg__MapPoint * value =
    (const yolo_data__msg__MapPoint *)(untyped_value);
  *item = *value;
}

bool yolo_data__msg__MonitorData__rosidl_typesupport_introspection_c__resize_function__MonitorData__points(
  void * untyped_member, size_t size)
{
  yolo_data__msg__MapPoint__Sequence * member =
    (yolo_data__msg__MapPoint__Sequence *)(untyped_member);
  yolo_data__msg__MapPoint__Sequence__fini(member);
  return yolo_data__msg__MapPoint__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember yolo_data__msg__MonitorData__rosidl_typesupport_introspection_c__MonitorData_message_member_array[3] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(yolo_data__msg__MonitorData, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "boxes",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(yolo_data__msg__MonitorData, boxes),  // bytes offset in struct
    NULL,  // default value
    yolo_data__msg__MonitorData__rosidl_typesupport_introspection_c__size_function__MonitorData__boxes,  // size() function pointer
    yolo_data__msg__MonitorData__rosidl_typesupport_introspection_c__get_const_function__MonitorData__boxes,  // get_const(index) function pointer
    yolo_data__msg__MonitorData__rosidl_typesupport_introspection_c__get_function__MonitorData__boxes,  // get(index) function pointer
    yolo_data__msg__MonitorData__rosidl_typesupport_introspection_c__fetch_function__MonitorData__boxes,  // fetch(index, &value) function pointer
    yolo_data__msg__MonitorData__rosidl_typesupport_introspection_c__assign_function__MonitorData__boxes,  // assign(index, value) function pointer
    yolo_data__msg__MonitorData__rosidl_typesupport_introspection_c__resize_function__MonitorData__boxes  // resize(index) function pointer
  },
  {
    "points",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(yolo_data__msg__MonitorData, points),  // bytes offset in struct
    NULL,  // default value
    yolo_data__msg__MonitorData__rosidl_typesupport_introspection_c__size_function__MonitorData__points,  // size() function pointer
    yolo_data__msg__MonitorData__rosidl_typesupport_introspection_c__get_const_function__MonitorData__points,  // get_const(index) function pointer
    yolo_data__msg__MonitorData__rosidl_typesupport_introspection_c__get_function__MonitorData__points,  // get(index) function pointer
    yolo_data__msg__MonitorData__rosidl_typesupport_introspection_c__fetch_function__MonitorData__points,  // fetch(index, &value) function pointer
    yolo_data__msg__MonitorData__rosidl_typesupport_introspection_c__assign_function__MonitorData__points,  // assign(index, value) function pointer
    yolo_data__msg__MonitorData__rosidl_typesupport_introspection_c__resize_function__MonitorData__points  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers yolo_data__msg__MonitorData__rosidl_typesupport_introspection_c__MonitorData_message_members = {
  "yolo_data__msg",  // message namespace
  "MonitorData",  // message name
  3,  // number of fields
  sizeof(yolo_data__msg__MonitorData),
  yolo_data__msg__MonitorData__rosidl_typesupport_introspection_c__MonitorData_message_member_array,  // message members
  yolo_data__msg__MonitorData__rosidl_typesupport_introspection_c__MonitorData_init_function,  // function to initialize message memory (memory has to be allocated)
  yolo_data__msg__MonitorData__rosidl_typesupport_introspection_c__MonitorData_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t yolo_data__msg__MonitorData__rosidl_typesupport_introspection_c__MonitorData_message_type_support_handle = {
  0,
  &yolo_data__msg__MonitorData__rosidl_typesupport_introspection_c__MonitorData_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_yolo_data
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, yolo_data, msg, MonitorData)() {
  yolo_data__msg__MonitorData__rosidl_typesupport_introspection_c__MonitorData_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  yolo_data__msg__MonitorData__rosidl_typesupport_introspection_c__MonitorData_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, yolo_data, msg, Monitored)();
  yolo_data__msg__MonitorData__rosidl_typesupport_introspection_c__MonitorData_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, yolo_data, msg, MapPoint)();
  if (!yolo_data__msg__MonitorData__rosidl_typesupport_introspection_c__MonitorData_message_type_support_handle.typesupport_identifier) {
    yolo_data__msg__MonitorData__rosidl_typesupport_introspection_c__MonitorData_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &yolo_data__msg__MonitorData__rosidl_typesupport_introspection_c__MonitorData_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
