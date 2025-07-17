// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from yoloinference:msg/BoundingBoxes.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "yoloinference/msg/detail/bounding_boxes__rosidl_typesupport_introspection_c.h"
#include "yoloinference/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "yoloinference/msg/detail/bounding_boxes__functions.h"
#include "yoloinference/msg/detail/bounding_boxes__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `boxes`
#include "yoloinference/msg/bounding_box.h"
// Member `boxes`
#include "yoloinference/msg/detail/bounding_box__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void yoloinference__msg__BoundingBoxes__rosidl_typesupport_introspection_c__BoundingBoxes_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  yoloinference__msg__BoundingBoxes__init(message_memory);
}

void yoloinference__msg__BoundingBoxes__rosidl_typesupport_introspection_c__BoundingBoxes_fini_function(void * message_memory)
{
  yoloinference__msg__BoundingBoxes__fini(message_memory);
}

size_t yoloinference__msg__BoundingBoxes__rosidl_typesupport_introspection_c__size_function__BoundingBoxes__boxes(
  const void * untyped_member)
{
  const yoloinference__msg__BoundingBox__Sequence * member =
    (const yoloinference__msg__BoundingBox__Sequence *)(untyped_member);
  return member->size;
}

const void * yoloinference__msg__BoundingBoxes__rosidl_typesupport_introspection_c__get_const_function__BoundingBoxes__boxes(
  const void * untyped_member, size_t index)
{
  const yoloinference__msg__BoundingBox__Sequence * member =
    (const yoloinference__msg__BoundingBox__Sequence *)(untyped_member);
  return &member->data[index];
}

void * yoloinference__msg__BoundingBoxes__rosidl_typesupport_introspection_c__get_function__BoundingBoxes__boxes(
  void * untyped_member, size_t index)
{
  yoloinference__msg__BoundingBox__Sequence * member =
    (yoloinference__msg__BoundingBox__Sequence *)(untyped_member);
  return &member->data[index];
}

void yoloinference__msg__BoundingBoxes__rosidl_typesupport_introspection_c__fetch_function__BoundingBoxes__boxes(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const yoloinference__msg__BoundingBox * item =
    ((const yoloinference__msg__BoundingBox *)
    yoloinference__msg__BoundingBoxes__rosidl_typesupport_introspection_c__get_const_function__BoundingBoxes__boxes(untyped_member, index));
  yoloinference__msg__BoundingBox * value =
    (yoloinference__msg__BoundingBox *)(untyped_value);
  *value = *item;
}

void yoloinference__msg__BoundingBoxes__rosidl_typesupport_introspection_c__assign_function__BoundingBoxes__boxes(
  void * untyped_member, size_t index, const void * untyped_value)
{
  yoloinference__msg__BoundingBox * item =
    ((yoloinference__msg__BoundingBox *)
    yoloinference__msg__BoundingBoxes__rosidl_typesupport_introspection_c__get_function__BoundingBoxes__boxes(untyped_member, index));
  const yoloinference__msg__BoundingBox * value =
    (const yoloinference__msg__BoundingBox *)(untyped_value);
  *item = *value;
}

bool yoloinference__msg__BoundingBoxes__rosidl_typesupport_introspection_c__resize_function__BoundingBoxes__boxes(
  void * untyped_member, size_t size)
{
  yoloinference__msg__BoundingBox__Sequence * member =
    (yoloinference__msg__BoundingBox__Sequence *)(untyped_member);
  yoloinference__msg__BoundingBox__Sequence__fini(member);
  return yoloinference__msg__BoundingBox__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember yoloinference__msg__BoundingBoxes__rosidl_typesupport_introspection_c__BoundingBoxes_message_member_array[2] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(yoloinference__msg__BoundingBoxes, header),  // bytes offset in struct
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
    offsetof(yoloinference__msg__BoundingBoxes, boxes),  // bytes offset in struct
    NULL,  // default value
    yoloinference__msg__BoundingBoxes__rosidl_typesupport_introspection_c__size_function__BoundingBoxes__boxes,  // size() function pointer
    yoloinference__msg__BoundingBoxes__rosidl_typesupport_introspection_c__get_const_function__BoundingBoxes__boxes,  // get_const(index) function pointer
    yoloinference__msg__BoundingBoxes__rosidl_typesupport_introspection_c__get_function__BoundingBoxes__boxes,  // get(index) function pointer
    yoloinference__msg__BoundingBoxes__rosidl_typesupport_introspection_c__fetch_function__BoundingBoxes__boxes,  // fetch(index, &value) function pointer
    yoloinference__msg__BoundingBoxes__rosidl_typesupport_introspection_c__assign_function__BoundingBoxes__boxes,  // assign(index, value) function pointer
    yoloinference__msg__BoundingBoxes__rosidl_typesupport_introspection_c__resize_function__BoundingBoxes__boxes  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers yoloinference__msg__BoundingBoxes__rosidl_typesupport_introspection_c__BoundingBoxes_message_members = {
  "yoloinference__msg",  // message namespace
  "BoundingBoxes",  // message name
  2,  // number of fields
  sizeof(yoloinference__msg__BoundingBoxes),
  yoloinference__msg__BoundingBoxes__rosidl_typesupport_introspection_c__BoundingBoxes_message_member_array,  // message members
  yoloinference__msg__BoundingBoxes__rosidl_typesupport_introspection_c__BoundingBoxes_init_function,  // function to initialize message memory (memory has to be allocated)
  yoloinference__msg__BoundingBoxes__rosidl_typesupport_introspection_c__BoundingBoxes_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t yoloinference__msg__BoundingBoxes__rosidl_typesupport_introspection_c__BoundingBoxes_message_type_support_handle = {
  0,
  &yoloinference__msg__BoundingBoxes__rosidl_typesupport_introspection_c__BoundingBoxes_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_yoloinference
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, yoloinference, msg, BoundingBoxes)() {
  yoloinference__msg__BoundingBoxes__rosidl_typesupport_introspection_c__BoundingBoxes_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  yoloinference__msg__BoundingBoxes__rosidl_typesupport_introspection_c__BoundingBoxes_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, yoloinference, msg, BoundingBox)();
  if (!yoloinference__msg__BoundingBoxes__rosidl_typesupport_introspection_c__BoundingBoxes_message_type_support_handle.typesupport_identifier) {
    yoloinference__msg__BoundingBoxes__rosidl_typesupport_introspection_c__BoundingBoxes_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &yoloinference__msg__BoundingBoxes__rosidl_typesupport_introspection_c__BoundingBoxes_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
