// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from yoloinference:msg/BoundingBox.idl
// generated code does not contain a copyright notice

#ifndef YOLOINFERENCE__MSG__DETAIL__BOUNDING_BOX__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define YOLOINFERENCE__MSG__DETAIL__BOUNDING_BOX__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "yoloinference/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "yoloinference/msg/detail/bounding_box__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace yoloinference
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_yoloinference
cdr_serialize(
  const yoloinference::msg::BoundingBox & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_yoloinference
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  yoloinference::msg::BoundingBox & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_yoloinference
get_serialized_size(
  const yoloinference::msg::BoundingBox & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_yoloinference
max_serialized_size_BoundingBox(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace yoloinference

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_yoloinference
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, yoloinference, msg, BoundingBox)();

#ifdef __cplusplus
}
#endif

#endif  // YOLOINFERENCE__MSG__DETAIL__BOUNDING_BOX__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
