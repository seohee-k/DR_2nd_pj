// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from yolo_data:msg/MonitorData.idl
// generated code does not contain a copyright notice

#ifndef YOLO_DATA__MSG__DETAIL__MONITOR_DATA__BUILDER_HPP_
#define YOLO_DATA__MSG__DETAIL__MONITOR_DATA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "yolo_data/msg/detail/monitor_data__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace yolo_data
{

namespace msg
{

namespace builder
{

class Init_MonitorData_points
{
public:
  explicit Init_MonitorData_points(::yolo_data::msg::MonitorData & msg)
  : msg_(msg)
  {}
  ::yolo_data::msg::MonitorData points(::yolo_data::msg::MonitorData::_points_type arg)
  {
    msg_.points = std::move(arg);
    return std::move(msg_);
  }

private:
  ::yolo_data::msg::MonitorData msg_;
};

class Init_MonitorData_boxes
{
public:
  explicit Init_MonitorData_boxes(::yolo_data::msg::MonitorData & msg)
  : msg_(msg)
  {}
  Init_MonitorData_points boxes(::yolo_data::msg::MonitorData::_boxes_type arg)
  {
    msg_.boxes = std::move(arg);
    return Init_MonitorData_points(msg_);
  }

private:
  ::yolo_data::msg::MonitorData msg_;
};

class Init_MonitorData_header
{
public:
  Init_MonitorData_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MonitorData_boxes header(::yolo_data::msg::MonitorData::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_MonitorData_boxes(msg_);
  }

private:
  ::yolo_data::msg::MonitorData msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::yolo_data::msg::MonitorData>()
{
  return yolo_data::msg::builder::Init_MonitorData_header();
}

}  // namespace yolo_data

#endif  // YOLO_DATA__MSG__DETAIL__MONITOR_DATA__BUILDER_HPP_
