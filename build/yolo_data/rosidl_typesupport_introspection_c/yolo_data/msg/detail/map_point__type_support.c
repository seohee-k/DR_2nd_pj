// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from yolo_data:msg/MapPoint.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "yolo_data/msg/detail/map_point__rosidl_typesupport_introspection_c.h"
#include "yolo_data/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "yolo_data/msg/detail/map_point__functions.h"
#include "yolo_data/msg/detail/map_point__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void yolo_data__msg__MapPoint__rosidl_typesupport_introspection_c__MapPoint_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  yolo_data__msg__MapPoint__init(message_memory);
}

void yolo_data__msg__MapPoint__rosidl_typesupport_introspection_c__MapPoint_fini_function(void * message_memory)
{
  yolo_data__msg__MapPoint__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember yolo_data__msg__MapPoint__rosidl_typesupport_introspection_c__MapPoint_message_member_array[3] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(yolo_data__msg__MapPoint, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "x",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(yolo_data__msg__MapPoint, x),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "y",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(yolo_data__msg__MapPoint, y),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers yolo_data__msg__MapPoint__rosidl_typesupport_introspection_c__MapPoint_message_members = {
  "yolo_data__msg",  // message namespace
  "MapPoint",  // message name
  3,  // number of fields
  sizeof(yolo_data__msg__MapPoint),
  yolo_data__msg__MapPoint__rosidl_typesupport_introspection_c__MapPoint_message_member_array,  // message members
  yolo_data__msg__MapPoint__rosidl_typesupport_introspection_c__MapPoint_init_function,  // function to initialize message memory (memory has to be allocated)
  yolo_data__msg__MapPoint__rosidl_typesupport_introspection_c__MapPoint_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t yolo_data__msg__MapPoint__rosidl_typesupport_introspection_c__MapPoint_message_type_support_handle = {
  0,
  &yolo_data__msg__MapPoint__rosidl_typesupport_introspection_c__MapPoint_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_yolo_data
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, yolo_data, msg, MapPoint)() {
  yolo_data__msg__MapPoint__rosidl_typesupport_introspection_c__MapPoint_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!yolo_data__msg__MapPoint__rosidl_typesupport_introspection_c__MapPoint_message_type_support_handle.typesupport_identifier) {
    yolo_data__msg__MapPoint__rosidl_typesupport_introspection_c__MapPoint_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &yolo_data__msg__MapPoint__rosidl_typesupport_introspection_c__MapPoint_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
