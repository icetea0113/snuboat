// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from snumsg_pkg:msg/Sensor.idl
// generated code does not contain a copyright notice

#ifndef SNUMSG_PKG__MSG__DETAIL__SENSOR__TRAITS_HPP_
#define SNUMSG_PKG__MSG__DETAIL__SENSOR__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "snumsg_pkg/msg/detail/sensor__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'tick'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace snumsg_pkg
{

namespace msg
{

inline void to_flow_style_yaml(
  const Sensor & msg,
  std::ostream & out)
{
  out << "{";
  // member: tick
  {
    out << "tick: ";
    to_flow_style_yaml(msg.tick, out);
    out << ", ";
  }

  // member: status
  {
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << ", ";
  }

  // member: pose
  {
    if (msg.pose.size() == 0) {
      out << "pose: []";
    } else {
      out << "pose: [";
      size_t pending_items = msg.pose.size();
      for (auto item : msg.pose) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: vel
  {
    if (msg.vel.size() == 0) {
      out << "vel: []";
    } else {
      out << "vel: [";
      size_t pending_items = msg.vel.size();
      for (auto item : msg.vel) {
        rosidl_generator_traits::value_to_yaml(item, out);
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
  const Sensor & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: tick
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "tick:\n";
    to_block_style_yaml(msg.tick, out, indentation + 2);
  }

  // member: status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << "\n";
  }

  // member: pose
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.pose.size() == 0) {
      out << "pose: []\n";
    } else {
      out << "pose:\n";
      for (auto item : msg.pose) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: vel
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.vel.size() == 0) {
      out << "vel: []\n";
    } else {
      out << "vel:\n";
      for (auto item : msg.vel) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Sensor & msg, bool use_flow_style = false)
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
  const snumsg_pkg::msg::Sensor & msg,
  std::ostream & out, size_t indentation = 0)
{
  snumsg_pkg::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use snumsg_pkg::msg::to_yaml() instead")]]
inline std::string to_yaml(const snumsg_pkg::msg::Sensor & msg)
{
  return snumsg_pkg::msg::to_yaml(msg);
}

template<>
inline const char * data_type<snumsg_pkg::msg::Sensor>()
{
  return "snumsg_pkg::msg::Sensor";
}

template<>
inline const char * name<snumsg_pkg::msg::Sensor>()
{
  return "snumsg_pkg/msg/Sensor";
}

template<>
struct has_fixed_size<snumsg_pkg::msg::Sensor>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<snumsg_pkg::msg::Sensor>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<snumsg_pkg::msg::Sensor>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // SNUMSG_PKG__MSG__DETAIL__SENSOR__TRAITS_HPP_
