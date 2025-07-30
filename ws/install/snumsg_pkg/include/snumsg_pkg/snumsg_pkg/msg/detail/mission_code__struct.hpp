// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from snumsg_pkg:msg/MissionCode.idl
// generated code does not contain a copyright notice

#ifndef SNUMSG_PKG__MSG__DETAIL__MISSION_CODE__STRUCT_HPP_
#define SNUMSG_PKG__MSG__DETAIL__MISSION_CODE__STRUCT_HPP_

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
# define DEPRECATED__snumsg_pkg__msg__MissionCode __attribute__((deprecated))
#else
# define DEPRECATED__snumsg_pkg__msg__MissionCode __declspec(deprecated)
#endif

namespace snumsg_pkg
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MissionCode_
{
  using Type = MissionCode_<ContainerAllocator>;

  explicit MissionCode_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : tick(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->mission_code = "";
    }
  }

  explicit MissionCode_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : tick(_alloc, _init),
    mission_code(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->mission_code = "";
    }
  }

  // field types and members
  using _tick_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _tick_type tick;
  using _mission_code_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _mission_code_type mission_code;

  // setters for named parameter idiom
  Type & set__tick(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->tick = _arg;
    return *this;
  }
  Type & set__mission_code(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->mission_code = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    snumsg_pkg::msg::MissionCode_<ContainerAllocator> *;
  using ConstRawPtr =
    const snumsg_pkg::msg::MissionCode_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<snumsg_pkg::msg::MissionCode_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<snumsg_pkg::msg::MissionCode_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      snumsg_pkg::msg::MissionCode_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<snumsg_pkg::msg::MissionCode_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      snumsg_pkg::msg::MissionCode_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<snumsg_pkg::msg::MissionCode_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<snumsg_pkg::msg::MissionCode_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<snumsg_pkg::msg::MissionCode_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__snumsg_pkg__msg__MissionCode
    std::shared_ptr<snumsg_pkg::msg::MissionCode_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__snumsg_pkg__msg__MissionCode
    std::shared_ptr<snumsg_pkg::msg::MissionCode_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MissionCode_ & other) const
  {
    if (this->tick != other.tick) {
      return false;
    }
    if (this->mission_code != other.mission_code) {
      return false;
    }
    return true;
  }
  bool operator!=(const MissionCode_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MissionCode_

// alias to use template instance with default allocator
using MissionCode =
  snumsg_pkg::msg::MissionCode_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace snumsg_pkg

#endif  // SNUMSG_PKG__MSG__DETAIL__MISSION_CODE__STRUCT_HPP_
