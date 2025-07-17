// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from yolo_data:msg/Monitored.idl
// generated code does not contain a copyright notice

#ifndef YOLO_DATA__MSG__DETAIL__MONITORED__BUILDER_HPP_
#define YOLO_DATA__MSG__DETAIL__MONITORED__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "yolo_data/msg/detail/monitored__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace yolo_data
{

namespace msg
{

namespace builder
{

class Init_Monitored_confidence
{
public:
  explicit Init_Monitored_confidence(::yolo_data::msg::Monitored & msg)
  : msg_(msg)
  {}
  ::yolo_data::msg::Monitored confidence(::yolo_data::msg::Monitored::_confidence_type arg)
  {
    msg_.confidence = std::move(arg);
    return std::move(msg_);
  }

private:
  ::yolo_data::msg::Monitored msg_;
};

class Init_Monitored_class_name
{
public:
  Init_Monitored_class_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Monitored_confidence class_name(::yolo_data::msg::Monitored::_class_name_type arg)
  {
    msg_.class_name = std::move(arg);
    return Init_Monitored_confidence(msg_);
  }

private:
  ::yolo_data::msg::Monitored msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::yolo_data::msg::Monitored>()
{
  return yolo_data::msg::builder::Init_Monitored_class_name();
}

}  // namespace yolo_data

#endif  // YOLO_DATA__MSG__DETAIL__MONITORED__BUILDER_HPP_
