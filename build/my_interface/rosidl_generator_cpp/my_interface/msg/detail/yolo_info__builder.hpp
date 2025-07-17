// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from my_interface:msg/YoloInfo.idl
// generated code does not contain a copyright notice

#ifndef MY_INTERFACE__MSG__DETAIL__YOLO_INFO__BUILDER_HPP_
#define MY_INTERFACE__MSG__DETAIL__YOLO_INFO__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "my_interface/msg/detail/yolo_info__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace my_interface
{

namespace msg
{

namespace builder
{

class Init_YoloInfo_y
{
public:
  explicit Init_YoloInfo_y(::my_interface::msg::YoloInfo & msg)
  : msg_(msg)
  {}
  ::my_interface::msg::YoloInfo y(::my_interface::msg::YoloInfo::_y_type arg)
  {
    msg_.y = std::move(arg);
    return std::move(msg_);
  }

private:
  ::my_interface::msg::YoloInfo msg_;
};

class Init_YoloInfo_x
{
public:
  explicit Init_YoloInfo_x(::my_interface::msg::YoloInfo & msg)
  : msg_(msg)
  {}
  Init_YoloInfo_y x(::my_interface::msg::YoloInfo::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_YoloInfo_y(msg_);
  }

private:
  ::my_interface::msg::YoloInfo msg_;
};

class Init_YoloInfo_confidence
{
public:
  explicit Init_YoloInfo_confidence(::my_interface::msg::YoloInfo & msg)
  : msg_(msg)
  {}
  Init_YoloInfo_x confidence(::my_interface::msg::YoloInfo::_confidence_type arg)
  {
    msg_.confidence = std::move(arg);
    return Init_YoloInfo_x(msg_);
  }

private:
  ::my_interface::msg::YoloInfo msg_;
};

class Init_YoloInfo_class_name
{
public:
  explicit Init_YoloInfo_class_name(::my_interface::msg::YoloInfo & msg)
  : msg_(msg)
  {}
  Init_YoloInfo_confidence class_name(::my_interface::msg::YoloInfo::_class_name_type arg)
  {
    msg_.class_name = std::move(arg);
    return Init_YoloInfo_confidence(msg_);
  }

private:
  ::my_interface::msg::YoloInfo msg_;
};

class Init_YoloInfo_header
{
public:
  Init_YoloInfo_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_YoloInfo_class_name header(::my_interface::msg::YoloInfo::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_YoloInfo_class_name(msg_);
  }

private:
  ::my_interface::msg::YoloInfo msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::my_interface::msg::YoloInfo>()
{
  return my_interface::msg::builder::Init_YoloInfo_header();
}

}  // namespace my_interface

#endif  // MY_INTERFACE__MSG__DETAIL__YOLO_INFO__BUILDER_HPP_
