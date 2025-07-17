// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from yolo_data:msg/MapPoint.idl
// generated code does not contain a copyright notice

#ifndef YOLO_DATA__MSG__DETAIL__MAP_POINT__BUILDER_HPP_
#define YOLO_DATA__MSG__DETAIL__MAP_POINT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "yolo_data/msg/detail/map_point__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace yolo_data
{

namespace msg
{

namespace builder
{

class Init_MapPoint_y
{
public:
  explicit Init_MapPoint_y(::yolo_data::msg::MapPoint & msg)
  : msg_(msg)
  {}
  ::yolo_data::msg::MapPoint y(::yolo_data::msg::MapPoint::_y_type arg)
  {
    msg_.y = std::move(arg);
    return std::move(msg_);
  }

private:
  ::yolo_data::msg::MapPoint msg_;
};

class Init_MapPoint_x
{
public:
  explicit Init_MapPoint_x(::yolo_data::msg::MapPoint & msg)
  : msg_(msg)
  {}
  Init_MapPoint_y x(::yolo_data::msg::MapPoint::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_MapPoint_y(msg_);
  }

private:
  ::yolo_data::msg::MapPoint msg_;
};

class Init_MapPoint_header
{
public:
  Init_MapPoint_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MapPoint_x header(::yolo_data::msg::MapPoint::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_MapPoint_x(msg_);
  }

private:
  ::yolo_data::msg::MapPoint msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::yolo_data::msg::MapPoint>()
{
  return yolo_data::msg::builder::Init_MapPoint_header();
}

}  // namespace yolo_data

#endif  // YOLO_DATA__MSG__DETAIL__MAP_POINT__BUILDER_HPP_
