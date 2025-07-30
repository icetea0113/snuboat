// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from snumsg_pkg:msg/Sensor.idl
// generated code does not contain a copyright notice

#ifndef SNUMSG_PKG__MSG__DETAIL__SENSOR__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define SNUMSG_PKG__MSG__DETAIL__SENSOR__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "snumsg_pkg/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "snumsg_pkg/msg/detail/sensor__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace snumsg_pkg
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_snumsg_pkg
cdr_serialize(
  const snumsg_pkg::msg::Sensor & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_snumsg_pkg
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  snumsg_pkg::msg::Sensor & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_snumsg_pkg
get_serialized_size(
  const snumsg_pkg::msg::Sensor & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_snumsg_pkg
max_serialized_size_Sensor(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace snumsg_pkg

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_snumsg_pkg
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, snumsg_pkg, msg, Sensor)();

#ifdef __cplusplus
}
#endif

#endif  // SNUMSG_PKG__MSG__DETAIL__SENSOR__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
