// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from yoloinference:msg/BoundingBoxes.idl
// generated code does not contain a copyright notice

#ifndef YOLOINFERENCE__MSG__DETAIL__BOUNDING_BOXES__BUILDER_HPP_
#define YOLOINFERENCE__MSG__DETAIL__BOUNDING_BOXES__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "yoloinference/msg/detail/bounding_boxes__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace yoloinference
{

namespace msg
{

namespace builder
{

class Init_BoundingBoxes_boxes
{
public:
  explicit Init_BoundingBoxes_boxes(::yoloinference::msg::BoundingBoxes & msg)
  : msg_(msg)
  {}
  ::yoloinference::msg::BoundingBoxes boxes(::yoloinference::msg::BoundingBoxes::_boxes_type arg)
  {
    msg_.boxes = std::move(arg);
    return std::move(msg_);
  }

private:
  ::yoloinference::msg::BoundingBoxes msg_;
};

class Init_BoundingBoxes_header
{
public:
  Init_BoundingBoxes_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_BoundingBoxes_boxes header(::yoloinference::msg::BoundingBoxes::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_BoundingBoxes_boxes(msg_);
  }

private:
  ::yoloinference::msg::BoundingBoxes msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::yoloinference::msg::BoundingBoxes>()
{
  return yoloinference::msg::builder::Init_BoundingBoxes_header();
}

}  // namespace yoloinference

#endif  // YOLOINFERENCE__MSG__DETAIL__BOUNDING_BOXES__BUILDER_HPP_
