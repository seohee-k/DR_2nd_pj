// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from yolo_data:msg/MonitorData.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "yolo_data/msg/detail/monitor_data__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace yolo_data
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void MonitorData_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) yolo_data::msg::MonitorData(_init);
}

void MonitorData_fini_function(void * message_memory)
{
  auto typed_message = static_cast<yolo_data::msg::MonitorData *>(message_memory);
  typed_message->~MonitorData();
}

size_t size_function__MonitorData__boxes(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<yolo_data::msg::Monitored> *>(untyped_member);
  return member->size();
}

const void * get_const_function__MonitorData__boxes(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<yolo_data::msg::Monitored> *>(untyped_member);
  return &member[index];
}

void * get_function__MonitorData__boxes(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<yolo_data::msg::Monitored> *>(untyped_member);
  return &member[index];
}

void fetch_function__MonitorData__boxes(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const yolo_data::msg::Monitored *>(
    get_const_function__MonitorData__boxes(untyped_member, index));
  auto & value = *reinterpret_cast<yolo_data::msg::Monitored *>(untyped_value);
  value = item;
}

void assign_function__MonitorData__boxes(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<yolo_data::msg::Monitored *>(
    get_function__MonitorData__boxes(untyped_member, index));
  const auto & value = *reinterpret_cast<const yolo_data::msg::Monitored *>(untyped_value);
  item = value;
}

void resize_function__MonitorData__boxes(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<yolo_data::msg::Monitored> *>(untyped_member);
  member->resize(size);
}

size_t size_function__MonitorData__points(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<yolo_data::msg::MapPoint> *>(untyped_member);
  return member->size();
}

const void * get_const_function__MonitorData__points(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<yolo_data::msg::MapPoint> *>(untyped_member);
  return &member[index];
}

void * get_function__MonitorData__points(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<yolo_data::msg::MapPoint> *>(untyped_member);
  return &member[index];
}

void fetch_function__MonitorData__points(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const yolo_data::msg::MapPoint *>(
    get_const_function__MonitorData__points(untyped_member, index));
  auto & value = *reinterpret_cast<yolo_data::msg::MapPoint *>(untyped_value);
  value = item;
}

void assign_function__MonitorData__points(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<yolo_data::msg::MapPoint *>(
    get_function__MonitorData__points(untyped_member, index));
  const auto & value = *reinterpret_cast<const yolo_data::msg::MapPoint *>(untyped_value);
  item = value;
}

void resize_function__MonitorData__points(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<yolo_data::msg::MapPoint> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember MonitorData_message_member_array[3] = {
  {
    "header",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Header>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(yolo_data::msg::MonitorData, header),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "boxes",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<yolo_data::msg::Monitored>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(yolo_data::msg::MonitorData, boxes),  // bytes offset in struct
    nullptr,  // default value
    size_function__MonitorData__boxes,  // size() function pointer
    get_const_function__MonitorData__boxes,  // get_const(index) function pointer
    get_function__MonitorData__boxes,  // get(index) function pointer
    fetch_function__MonitorData__boxes,  // fetch(index, &value) function pointer
    assign_function__MonitorData__boxes,  // assign(index, value) function pointer
    resize_function__MonitorData__boxes  // resize(index) function pointer
  },
  {
    "points",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<yolo_data::msg::MapPoint>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(yolo_data::msg::MonitorData, points),  // bytes offset in struct
    nullptr,  // default value
    size_function__MonitorData__points,  // size() function pointer
    get_const_function__MonitorData__points,  // get_const(index) function pointer
    get_function__MonitorData__points,  // get(index) function pointer
    fetch_function__MonitorData__points,  // fetch(index, &value) function pointer
    assign_function__MonitorData__points,  // assign(index, value) function pointer
    resize_function__MonitorData__points  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers MonitorData_message_members = {
  "yolo_data::msg",  // message namespace
  "MonitorData",  // message name
  3,  // number of fields
  sizeof(yolo_data::msg::MonitorData),
  MonitorData_message_member_array,  // message members
  MonitorData_init_function,  // function to initialize message memory (memory has to be allocated)
  MonitorData_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t MonitorData_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &MonitorData_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace yolo_data


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<yolo_data::msg::MonitorData>()
{
  return &::yolo_data::msg::rosidl_typesupport_introspection_cpp::MonitorData_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, yolo_data, msg, MonitorData)() {
  return &::yolo_data::msg::rosidl_typesupport_introspection_cpp::MonitorData_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
