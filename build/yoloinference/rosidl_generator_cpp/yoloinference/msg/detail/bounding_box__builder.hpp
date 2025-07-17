// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from yoloinference:msg/BoundingBox.idl
// generated code does not contain a copyright notice

#ifndef YOLOINFERENCE__MSG__DETAIL__BOUNDING_BOX__BUILDER_HPP_
#define YOLOINFERENCE__MSG__DETAIL__BOUNDING_BOX__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "yoloinference/msg/detail/bounding_box__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace yoloinference
{

namespace msg
{

namespace builder
{

class Init_BoundingBox_y2
{
public:
  explicit Init_BoundingBox_y2(::yoloinference::msg::BoundingBox & msg)
  : msg_(msg)
  {}
  ::yoloinference::msg::BoundingBox y2(::yoloinference::msg::BoundingBox::_y2_type arg)
  {
    msg_.y2 = std::move(arg);
    return std::move(msg_);
  }

private:
  ::yoloinference::msg::BoundingBox msg_;
};

class Init_BoundingBox_x2
{
public:
  explicit Init_BoundingBox_x2(::yoloinference::msg::BoundingBox & msg)
  : msg_(msg)
  {}
  Init_BoundingBox_y2 x2(::yoloinference::msg::BoundingBox::_x2_type arg)
  {
    msg_.x2 = std::move(arg);
    return Init_BoundingBox_y2(msg_);
  }

private:
  ::yoloinference::msg::BoundingBox msg_;
};

class Init_BoundingBox_y1
{
public:
  explicit Init_BoundingBox_y1(::yoloinference::msg::BoundingBox & msg)
  : msg_(msg)
  {}
  Init_BoundingBox_x2 y1(::yoloinference::msg::BoundingBox::_y1_type arg)
  {
    msg_.y1 = std::move(arg);
    return Init_BoundingBox_x2(msg_);
  }

private:
  ::yoloinference::msg::BoundingBox msg_;
};

class Init_BoundingBox_x1
{
public:
  explicit Init_BoundingBox_x1(::yoloinference::msg::BoundingBox & msg)
  : msg_(msg)
  {}
  Init_BoundingBox_y1 x1(::yoloinference::msg::BoundingBox::_x1_type arg)
  {
    msg_.x1 = std::move(arg);
    return Init_BoundingBox_y1(msg_);
  }

private:
  ::yoloinference::msg::BoundingBox msg_;
};

class Init_BoundingBox_confidence
{
public:
  explicit Init_BoundingBox_confidence(::yoloinference::msg::BoundingBox & msg)
  : msg_(msg)
  {}
  Init_BoundingBox_x1 confidence(::yoloinference::msg::BoundingBox::_confidence_type arg)
  {
    msg_.confidence = std::move(arg);
    return Init_BoundingBox_x1(msg_);
  }

private:
  ::yoloinference::msg::BoundingBox msg_;
};

class Init_BoundingBox_class_name
{
public:
  Init_BoundingBox_class_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_BoundingBox_confidence class_name(::yoloinference::msg::BoundingBox::_class_name_type arg)
  {
    msg_.class_name = std::move(arg);
    return Init_BoundingBox_confidence(msg_);
  }

private:
  ::yoloinference::msg::BoundingBox msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::yoloinference::msg::BoundingBox>()
{
  return yoloinference::msg::builder::Init_BoundingBox_class_name();
}

}  // namespace yoloinference

#endif  // YOLOINFERENCE__MSG__DETAIL__BOUNDING_BOX__BUILDER_HPP_
