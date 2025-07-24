// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from snumsg_pkg:msg/MissionCode.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "snumsg_pkg/msg/detail/mission_code__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace snumsg_pkg
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void MissionCode_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) snumsg_pkg::msg::MissionCode(_init);
}

void MissionCode_fini_function(void * message_memory)
{
  auto typed_message = static_cast<snumsg_pkg::msg::MissionCode *>(message_memory);
  typed_message->~MissionCode();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember MissionCode_message_member_array[2] = {
  {
    "timestamp",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<builtin_interfaces::msg::Time>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(snumsg_pkg::msg::MissionCode, timestamp),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "value",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(snumsg_pkg::msg::MissionCode, value),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers MissionCode_message_members = {
  "snumsg_pkg::msg",  // message namespace
  "MissionCode",  // message name
  2,  // number of fields
  sizeof(snumsg_pkg::msg::MissionCode),
  MissionCode_message_member_array,  // message members
  MissionCode_init_function,  // function to initialize message memory (memory has to be allocated)
  MissionCode_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t MissionCode_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &MissionCode_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace snumsg_pkg


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<snumsg_pkg::msg::MissionCode>()
{
  return &::snumsg_pkg::msg::rosidl_typesupport_introspection_cpp::MissionCode_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, snumsg_pkg, msg, MissionCode)() {
  return &::snumsg_pkg::msg::rosidl_typesupport_introspection_cpp::MissionCode_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
