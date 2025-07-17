// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from yolo_data:msg/Monitored.idl
// generated code does not contain a copyright notice

#ifndef YOLO_DATA__MSG__DETAIL__MONITORED__STRUCT_HPP_
#define YOLO_DATA__MSG__DETAIL__MONITORED__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__yolo_data__msg__Monitored __attribute__((deprecated))
#else
# define DEPRECATED__yolo_data__msg__Monitored __declspec(deprecated)
#endif

namespace yolo_data
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Monitored_
{
  using Type = Monitored_<ContainerAllocator>;

  explicit Monitored_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->class_name = "";
      this->confidence = 0.0f;
    }
  }

  explicit Monitored_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : class_name(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->class_name = "";
      this->confidence = 0.0f;
    }
  }

  // field types and members
  using _class_name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _class_name_type class_name;
  using _confidence_type =
    float;
  _confidence_type confidence;

  // setters for named parameter idiom
  Type & set__class_name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->class_name = _arg;
    return *this;
  }
  Type & set__confidence(
    const float & _arg)
  {
    this->confidence = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    yolo_data::msg::Monitored_<ContainerAllocator> *;
  using ConstRawPtr =
    const yolo_data::msg::Monitored_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<yolo_data::msg::Monitored_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<yolo_data::msg::Monitored_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      yolo_data::msg::Monitored_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<yolo_data::msg::Monitored_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      yolo_data::msg::Monitored_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<yolo_data::msg::Monitored_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<yolo_data::msg::Monitored_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<yolo_data::msg::Monitored_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__yolo_data__msg__Monitored
    std::shared_ptr<yolo_data::msg::Monitored_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__yolo_data__msg__Monitored
    std::shared_ptr<yolo_data::msg::Monitored_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Monitored_ & other) const
  {
    if (this->class_name != other.class_name) {
      return false;
    }
    if (this->confidence != other.confidence) {
      return false;
    }
    return true;
  }
  bool operator!=(const Monitored_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Monitored_

// alias to use template instance with default allocator
using Monitored =
  yolo_data::msg::Monitored_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace yolo_data

#endif  // YOLO_DATA__MSG__DETAIL__MONITORED__STRUCT_HPP_
