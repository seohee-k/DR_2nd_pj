// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from yolo_data:msg/MonitorData.idl
// generated code does not contain a copyright notice

#ifndef YOLO_DATA__MSG__DETAIL__MONITOR_DATA__TRAITS_HPP_
#define YOLO_DATA__MSG__DETAIL__MONITOR_DATA__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "yolo_data/msg/detail/monitor_data__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'boxes'
#include "yolo_data/msg/detail/monitored__traits.hpp"
// Member 'points'
#include "yolo_data/msg/detail/map_point__traits.hpp"

namespace yolo_data
{

namespace msg
{

inline void to_flow_style_yaml(
  const MonitorData & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: boxes
  {
    if (msg.boxes.size() == 0) {
      out << "boxes: []";
    } else {
      out << "boxes: [";
      size_t pending_items = msg.boxes.size();
      for (auto item : msg.boxes) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: points
  {
    if (msg.points.size() == 0) {
      out << "points: []";
    } else {
      out << "points: [";
      size_t pending_items = msg.points.size();
      for (auto item : msg.points) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MonitorData & msg,
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

  // member: boxes
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.boxes.size() == 0) {
      out << "boxes: []\n";
    } else {
      out << "boxes:\n";
      for (auto item : msg.boxes) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: points
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.points.size() == 0) {
      out << "points: []\n";
    } else {
      out << "points:\n";
      for (auto item : msg.points) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MonitorData & msg, bool use_flow_style = false)
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
  const yolo_data::msg::MonitorData & msg,
  std::ostream & out, size_t indentation = 0)
{
  yolo_data::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use yolo_data::msg::to_yaml() instead")]]
inline std::string to_yaml(const yolo_data::msg::MonitorData & msg)
{
  return yolo_data::msg::to_yaml(msg);
}

template<>
inline const char * data_type<yolo_data::msg::MonitorData>()
{
  return "yolo_data::msg::MonitorData";
}

template<>
inline const char * name<yolo_data::msg::MonitorData>()
{
  return "yolo_data/msg/MonitorData";
}

template<>
struct has_fixed_size<yolo_data::msg::MonitorData>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<yolo_data::msg::MonitorData>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<yolo_data::msg::MonitorData>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // YOLO_DATA__MSG__DETAIL__MONITOR_DATA__TRAITS_HPP_
