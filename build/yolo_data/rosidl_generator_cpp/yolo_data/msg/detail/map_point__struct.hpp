// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from yolo_data:msg/MapPoint.idl
// generated code does not contain a copyright notice

#ifndef YOLO_DATA__MSG__DETAIL__MAP_POINT__STRUCT_HPP_
#define YOLO_DATA__MSG__DETAIL__MAP_POINT__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__yolo_data__msg__MapPoint __attribute__((deprecated))
#else
# define DEPRECATED__yolo_data__msg__MapPoint __declspec(deprecated)
#endif

namespace yolo_data
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MapPoint_
{
  using Type = MapPoint_<ContainerAllocator>;

  explicit MapPoint_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x = 0.0f;
      this->y = 0.0f;
    }
  }

  explicit MapPoint_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x = 0.0f;
      this->y = 0.0f;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _x_type =
    float;
  _x_type x;
  using _y_type =
    float;
  _y_type y;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__x(
    const float & _arg)
  {
    this->x = _arg;
    return *this;
  }
  Type & set__y(
    const float & _arg)
  {
    this->y = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    yolo_data::msg::MapPoint_<ContainerAllocator> *;
  using ConstRawPtr =
    const yolo_data::msg::MapPoint_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<yolo_data::msg::MapPoint_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<yolo_data::msg::MapPoint_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      yolo_data::msg::MapPoint_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<yolo_data::msg::MapPoint_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      yolo_data::msg::MapPoint_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<yolo_data::msg::MapPoint_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<yolo_data::msg::MapPoint_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<yolo_data::msg::MapPoint_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__yolo_data__msg__MapPoint
    std::shared_ptr<yolo_data::msg::MapPoint_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__yolo_data__msg__MapPoint
    std::shared_ptr<yolo_data::msg::MapPoint_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MapPoint_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->x != other.x) {
      return false;
    }
    if (this->y != other.y) {
      return false;
    }
    return true;
  }
  bool operator!=(const MapPoint_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MapPoint_

// alias to use template instance with default allocator
using MapPoint =
  yolo_data::msg::MapPoint_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace yolo_data

#endif  // YOLO_DATA__MSG__DETAIL__MAP_POINT__STRUCT_HPP_
