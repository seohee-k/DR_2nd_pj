// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from my_interface:msg/YoloInfo.idl
// generated code does not contain a copyright notice

#ifndef MY_INTERFACE__MSG__DETAIL__YOLO_INFO__TRAITS_HPP_
#define MY_INTERFACE__MSG__DETAIL__YOLO_INFO__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "my_interface/msg/detail/yolo_info__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace my_interface
{

namespace msg
{

inline void to_flow_style_yaml(
  const YoloInfo & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: class_name
  {
    out << "class_name: ";
    rosidl_generator_traits::value_to_yaml(msg.class_name, out);
    out << ", ";
  }

  // member: confidence
  {
    out << "confidence: ";
    rosidl_generator_traits::value_to_yaml(msg.confidence, out);
    out << ", ";
  }

  // member: x
  {
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << ", ";
  }

  // member: y
  {
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const YoloInfo & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: class_name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "class_name: ";
    rosidl_generator_traits::value_to_yaml(msg.class_name, out);
    out << "\n";
  }

  // member: confidence
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "confidence: ";
    rosidl_generator_traits::value_to_yaml(msg.confidence, out);
    out << "\n";
  }

  // member: x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << "\n";
  }

  // member: y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const YoloInfo & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace my_interface

namespace rosidl_generator_traits
{

[[deprecated("use my_interface::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const my_interface::msg::YoloInfo & msg,
  std::ostream & out, size_t indentation = 0)
{
  my_interface::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use my_interface::msg::to_yaml() instead")]]
inline std::string to_yaml(const my_interface::msg::YoloInfo & msg)
{
  return my_interface::msg::to_yaml(msg);
}

template<>
inline const char * data_type<my_interface::msg::YoloInfo>()
{
  return "my_interface::msg::YoloInfo";
}

template<>
inline const char * name<my_interface::msg::YoloInfo>()
{
  return "my_interface/msg/YoloInfo";
}

template<>
struct has_fixed_size<my_interface::msg::YoloInfo>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<my_interface::msg::YoloInfo>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<my_interface::msg::YoloInfo>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MY_INTERFACE__MSG__DETAIL__YOLO_INFO__TRAITS_HPP_
