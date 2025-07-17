// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from yolo_data:msg/MonitorData.idl
// generated code does not contain a copyright notice

#ifndef YOLO_DATA__MSG__DETAIL__MONITOR_DATA__STRUCT_HPP_
#define YOLO_DATA__MSG__DETAIL__MONITOR_DATA__STRUCT_HPP_

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
// Member 'boxes'
#include "yolo_data/msg/detail/monitored__struct.hpp"
// Member 'points'
#include "yolo_data/msg/detail/map_point__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__yolo_data__msg__MonitorData __attribute__((deprecated))
#else
# define DEPRECATED__yolo_data__msg__MonitorData __declspec(deprecated)
#endif

namespace yolo_data
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MonitorData_
{
  using Type = MonitorData_<ContainerAllocator>;

  explicit MonitorData_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    (void)_init;
  }

  explicit MonitorData_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _boxes_type =
    std::vector<yolo_data::msg::Monitored_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<yolo_data::msg::Monitored_<ContainerAllocator>>>;
  _boxes_type boxes;
  using _points_type =
    std::vector<yolo_data::msg::MapPoint_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<yolo_data::msg::MapPoint_<ContainerAllocator>>>;
  _points_type points;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__boxes(
    const std::vector<yolo_data::msg::Monitored_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<yolo_data::msg::Monitored_<ContainerAllocator>>> & _arg)
  {
    this->boxes = _arg;
    return *this;
  }
  Type & set__points(
    const std::vector<yolo_data::msg::MapPoint_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<yolo_data::msg::MapPoint_<ContainerAllocator>>> & _arg)
  {
    this->points = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    yolo_data::msg::MonitorData_<ContainerAllocator> *;
  using ConstRawPtr =
    const yolo_data::msg::MonitorData_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<yolo_data::msg::MonitorData_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<yolo_data::msg::MonitorData_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      yolo_data::msg::MonitorData_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<yolo_data::msg::MonitorData_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      yolo_data::msg::MonitorData_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<yolo_data::msg::MonitorData_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<yolo_data::msg::MonitorData_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<yolo_data::msg::MonitorData_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__yolo_data__msg__MonitorData
    std::shared_ptr<yolo_data::msg::MonitorData_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__yolo_data__msg__MonitorData
    std::shared_ptr<yolo_data::msg::MonitorData_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MonitorData_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->boxes != other.boxes) {
      return false;
    }
    if (this->points != other.points) {
      return false;
    }
    return true;
  }
  bool operator!=(const MonitorData_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MonitorData_

// alias to use template instance with default allocator
using MonitorData =
  yolo_data::msg::MonitorData_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace yolo_data

#endif  // YOLO_DATA__MSG__DETAIL__MONITOR_DATA__STRUCT_HPP_
