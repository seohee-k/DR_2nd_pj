// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from my_interface:msg/YoloInfo.idl
// generated code does not contain a copyright notice

#ifndef MY_INTERFACE__MSG__DETAIL__YOLO_INFO__STRUCT_HPP_
#define MY_INTERFACE__MSG__DETAIL__YOLO_INFO__STRUCT_HPP_

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
# define DEPRECATED__my_interface__msg__YoloInfo __attribute__((deprecated))
#else
# define DEPRECATED__my_interface__msg__YoloInfo __declspec(deprecated)
#endif

namespace my_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct YoloInfo_
{
  using Type = YoloInfo_<ContainerAllocator>;

  explicit YoloInfo_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->class_name = "";
      this->confidence = 0.0;
      this->x = 0.0;
      this->y = 0.0;
    }
  }

  explicit YoloInfo_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    class_name(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->class_name = "";
      this->confidence = 0.0;
      this->x = 0.0;
      this->y = 0.0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _class_name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _class_name_type class_name;
  using _confidence_type =
    double;
  _confidence_type confidence;
  using _x_type =
    double;
  _x_type x;
  using _y_type =
    double;
  _y_type y;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__class_name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->class_name = _arg;
    return *this;
  }
  Type & set__confidence(
    const double & _arg)
  {
    this->confidence = _arg;
    return *this;
  }
  Type & set__x(
    const double & _arg)
  {
    this->x = _arg;
    return *this;
  }
  Type & set__y(
    const double & _arg)
  {
    this->y = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    my_interface::msg::YoloInfo_<ContainerAllocator> *;
  using ConstRawPtr =
    const my_interface::msg::YoloInfo_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<my_interface::msg::YoloInfo_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<my_interface::msg::YoloInfo_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      my_interface::msg::YoloInfo_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<my_interface::msg::YoloInfo_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      my_interface::msg::YoloInfo_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<my_interface::msg::YoloInfo_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<my_interface::msg::YoloInfo_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<my_interface::msg::YoloInfo_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__my_interface__msg__YoloInfo
    std::shared_ptr<my_interface::msg::YoloInfo_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__my_interface__msg__YoloInfo
    std::shared_ptr<my_interface::msg::YoloInfo_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const YoloInfo_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->class_name != other.class_name) {
      return false;
    }
    if (this->confidence != other.confidence) {
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
  bool operator!=(const YoloInfo_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct YoloInfo_

// alias to use template instance with default allocator
using YoloInfo =
  my_interface::msg::YoloInfo_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace my_interface

#endif  // MY_INTERFACE__MSG__DETAIL__YOLO_INFO__STRUCT_HPP_
