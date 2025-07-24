// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from snumsg_pkg:msg/MissionCode.idl
// generated code does not contain a copyright notice

#ifndef SNUMSG_PKG__MSG__DETAIL__MISSION_CODE__TRAITS_HPP_
#define SNUMSG_PKG__MSG__DETAIL__MISSION_CODE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "snumsg_pkg/msg/detail/mission_code__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'timestamp'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace snumsg_pkg
{

namespace msg
{

inline void to_flow_style_yaml(
  const MissionCode & msg,
  std::ostream & out)
{
  out << "{";
  // member: timestamp
  {
    out << "timestamp: ";
    to_flow_style_yaml(msg.timestamp, out);
    out << ", ";
  }

  // member: value
  {
    out << "value: ";
    rosidl_generator_traits::value_to_yaml(msg.value, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MissionCode & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: timestamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "timestamp:\n";
    to_block_style_yaml(msg.timestamp, out, indentation + 2);
  }

  // member: value
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "value: ";
    rosidl_generator_traits::value_to_yaml(msg.value, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MissionCode & msg, bool use_flow_style = false)
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

}  // namespace snumsg_pkg

namespace rosidl_generator_traits
{

[[deprecated("use snumsg_pkg::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const snumsg_pkg::msg::MissionCode & msg,
  std::ostream & out, size_t indentation = 0)
{
  snumsg_pkg::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use snumsg_pkg::msg::to_yaml() instead")]]
inline std::string to_yaml(const snumsg_pkg::msg::MissionCode & msg)
{
  return snumsg_pkg::msg::to_yaml(msg);
}

template<>
inline const char * data_type<snumsg_pkg::msg::MissionCode>()
{
  return "snumsg_pkg::msg::MissionCode";
}

template<>
inline const char * name<snumsg_pkg::msg::MissionCode>()
{
  return "snumsg_pkg/msg/MissionCode";
}

template<>
struct has_fixed_size<snumsg_pkg::msg::MissionCode>
  : std::integral_constant<bool, has_fixed_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct has_bounded_size<snumsg_pkg::msg::MissionCode>
  : std::integral_constant<bool, has_bounded_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct is_message<snumsg_pkg::msg::MissionCode>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // SNUMSG_PKG__MSG__DETAIL__MISSION_CODE__TRAITS_HPP_
