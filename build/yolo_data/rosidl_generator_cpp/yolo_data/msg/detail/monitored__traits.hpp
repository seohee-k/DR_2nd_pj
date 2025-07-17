// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from yolo_data:msg/Monitored.idl
// generated code does not contain a copyright notice

#ifndef YOLO_DATA__MSG__DETAIL__MONITORED__TRAITS_HPP_
#define YOLO_DATA__MSG__DETAIL__MONITORED__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "yolo_data/msg/detail/monitored__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace yolo_data
{

namespace msg
{

inline void to_flow_style_yaml(
  const Monitored & msg,
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
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Monitored & msg,
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
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Monitored & msg, bool use_flow_style = false)
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

}  // namespace yolo_data

namespace rosidl_generator_traits
{

[[deprecated("use yolo_data::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const yolo_data::msg::Monitored & msg,
  std::ostream & out, size_t indentation = 0)
{
  yolo_data::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use yolo_data::msg::to_yaml() instead")]]
inline std::string to_yaml(const yolo_data::msg::Monitored & msg)
{
  return yolo_data::msg::to_yaml(msg);
}

template<>
inline const char * data_type<yolo_data::msg::Monitored>()
{
  return "yolo_data::msg::Monitored";
}

template<>
inline const char * name<yolo_data::msg::Monitored>()
{
  return "yolo_data/msg/Monitored";
}

template<>
struct has_fixed_size<yolo_data::msg::Monitored>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<yolo_data::msg::Monitored>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<yolo_data::msg::Monitored>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // YOLO_DATA__MSG__DETAIL__MONITORED__TRAITS_HPP_
