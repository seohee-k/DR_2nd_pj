// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from yolo_data:msg/MapPoint.idl
// generated code does not contain a copyright notice

#ifndef YOLO_DATA__MSG__DETAIL__MAP_POINT__TRAITS_HPP_
#define YOLO_DATA__MSG__DETAIL__MAP_POINT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "yolo_data/msg/detail/map_point__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace yolo_data
{

namespace msg
{

inline void to_flow_style_yaml(
  const MapPoint & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
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
  const MapPoint & msg,
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

inline std::string to_yaml(const MapPoint & msg, bool use_flow_style = false)
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
  const yolo_data::msg::MapPoint & msg,
  std::ostream & out, size_t indentation = 0)
{
  yolo_data::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use yolo_data::msg::to_yaml() instead")]]
inline std::string to_yaml(const yolo_data::msg::MapPoint & msg)
{
  return yolo_data::msg::to_yaml(msg);
}

template<>
inline const char * data_type<yolo_data::msg::MapPoint>()
{
  return "yolo_data::msg::MapPoint";
}

template<>
inline const char * name<yolo_data::msg::MapPoint>()
{
  return "yolo_data/msg/MapPoint";
}

template<>
struct has_fixed_size<yolo_data::msg::MapPoint>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<yolo_data::msg::MapPoint>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<yolo_data::msg::MapPoint>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // YOLO_DATA__MSG__DETAIL__MAP_POINT__TRAITS_HPP_
