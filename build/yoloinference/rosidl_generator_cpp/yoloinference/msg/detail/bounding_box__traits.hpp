// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from yoloinference:msg/BoundingBox.idl
// generated code does not contain a copyright notice

#ifndef YOLOINFERENCE__MSG__DETAIL__BOUNDING_BOX__TRAITS_HPP_
#define YOLOINFERENCE__MSG__DETAIL__BOUNDING_BOX__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "yoloinference/msg/detail/bounding_box__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace yoloinference
{

namespace msg
{

inline void to_flow_style_yaml(
  const BoundingBox & msg,
  std::ostream & out)
{
  out << "{";
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

  // member: x1
  {
    out << "x1: ";
    rosidl_generator_traits::value_to_yaml(msg.x1, out);
    out << ", ";
  }

  // member: y1
  {
    out << "y1: ";
    rosidl_generator_traits::value_to_yaml(msg.y1, out);
    out << ", ";
  }

  // member: x2
  {
    out << "x2: ";
    rosidl_generator_traits::value_to_yaml(msg.x2, out);
    out << ", ";
  }

  // member: y2
  {
    out << "y2: ";
    rosidl_generator_traits::value_to_yaml(msg.y2, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const BoundingBox & msg,
  std::ostream & out, size_t indentation = 0)
{
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

  // member: x1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x1: ";
    rosidl_generator_traits::value_to_yaml(msg.x1, out);
    out << "\n";
  }

  // member: y1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y1: ";
    rosidl_generator_traits::value_to_yaml(msg.y1, out);
    out << "\n";
  }

  // member: x2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x2: ";
    rosidl_generator_traits::value_to_yaml(msg.x2, out);
    out << "\n";
  }

  // member: y2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y2: ";
    rosidl_generator_traits::value_to_yaml(msg.y2, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const BoundingBox & msg, bool use_flow_style = false)
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

}  // namespace yoloinference

namespace rosidl_generator_traits
{

[[deprecated("use yoloinference::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const yoloinference::msg::BoundingBox & msg,
  std::ostream & out, size_t indentation = 0)
{
  yoloinference::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use yoloinference::msg::to_yaml() instead")]]
inline std::string to_yaml(const yoloinference::msg::BoundingBox & msg)
{
  return yoloinference::msg::to_yaml(msg);
}

template<>
inline const char * data_type<yoloinference::msg::BoundingBox>()
{
  return "yoloinference::msg::BoundingBox";
}

template<>
inline const char * name<yoloinference::msg::BoundingBox>()
{
  return "yoloinference/msg/BoundingBox";
}

template<>
struct has_fixed_size<yoloinference::msg::BoundingBox>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<yoloinference::msg::BoundingBox>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<yoloinference::msg::BoundingBox>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // YOLOINFERENCE__MSG__DETAIL__BOUNDING_BOX__TRAITS_HPP_
