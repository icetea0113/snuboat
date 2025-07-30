// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from snumsg_pkg:msg/Sensor.idl
// generated code does not contain a copyright notice

#ifndef SNUMSG_PKG__MSG__DETAIL__SENSOR__STRUCT_HPP_
#define SNUMSG_PKG__MSG__DETAIL__SENSOR__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'tick'
#include "builtin_interfaces/msg/detail/time__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__snumsg_pkg__msg__Sensor __attribute__((deprecated))
#else
# define DEPRECATED__snumsg_pkg__msg__Sensor __declspec(deprecated)
#endif

namespace snumsg_pkg
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Sensor_
{
  using Type = Sensor_<ContainerAllocator>;

  explicit Sensor_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : tick(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = "";
    }
  }

  explicit Sensor_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : tick(_alloc, _init),
    status(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = "";
    }
  }

  // field types and members
  using _tick_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _tick_type tick;
  using _status_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _status_type status;
  using _pose_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _pose_type pose;
  using _vel_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _vel_type vel;

  // setters for named parameter idiom
  Type & set__tick(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->tick = _arg;
    return *this;
  }
  Type & set__status(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->status = _arg;
    return *this;
  }
  Type & set__pose(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->pose = _arg;
    return *this;
  }
  Type & set__vel(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->vel = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    snumsg_pkg::msg::Sensor_<ContainerAllocator> *;
  using ConstRawPtr =
    const snumsg_pkg::msg::Sensor_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<snumsg_pkg::msg::Sensor_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<snumsg_pkg::msg::Sensor_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      snumsg_pkg::msg::Sensor_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<snumsg_pkg::msg::Sensor_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      snumsg_pkg::msg::Sensor_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<snumsg_pkg::msg::Sensor_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<snumsg_pkg::msg::Sensor_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<snumsg_pkg::msg::Sensor_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__snumsg_pkg__msg__Sensor
    std::shared_ptr<snumsg_pkg::msg::Sensor_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__snumsg_pkg__msg__Sensor
    std::shared_ptr<snumsg_pkg::msg::Sensor_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Sensor_ & other) const
  {
    if (this->tick != other.tick) {
      return false;
    }
    if (this->status != other.status) {
      return false;
    }
    if (this->pose != other.pose) {
      return false;
    }
    if (this->vel != other.vel) {
      return false;
    }
    return true;
  }
  bool operator!=(const Sensor_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Sensor_

// alias to use template instance with default allocator
using Sensor =
  snumsg_pkg::msg::Sensor_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace snumsg_pkg

#endif  // SNUMSG_PKG__MSG__DETAIL__SENSOR__STRUCT_HPP_
