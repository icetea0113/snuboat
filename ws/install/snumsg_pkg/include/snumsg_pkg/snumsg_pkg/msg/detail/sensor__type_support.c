// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from snumsg_pkg:msg/Sensor.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "snumsg_pkg/msg/detail/sensor__rosidl_typesupport_introspection_c.h"
#include "snumsg_pkg/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "snumsg_pkg/msg/detail/sensor__functions.h"
#include "snumsg_pkg/msg/detail/sensor__struct.h"


// Include directives for member types
// Member `tick`
#include "builtin_interfaces/msg/time.h"
// Member `tick`
#include "builtin_interfaces/msg/detail/time__rosidl_typesupport_introspection_c.h"
// Member `status`
#include "rosidl_runtime_c/string_functions.h"
// Member `pose`
// Member `vel`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void snumsg_pkg__msg__Sensor__rosidl_typesupport_introspection_c__Sensor_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  snumsg_pkg__msg__Sensor__init(message_memory);
}

void snumsg_pkg__msg__Sensor__rosidl_typesupport_introspection_c__Sensor_fini_function(void * message_memory)
{
  snumsg_pkg__msg__Sensor__fini(message_memory);
}

size_t snumsg_pkg__msg__Sensor__rosidl_typesupport_introspection_c__size_function__Sensor__pose(
  const void * untyped_member)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return member->size;
}

const void * snumsg_pkg__msg__Sensor__rosidl_typesupport_introspection_c__get_const_function__Sensor__pose(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void * snumsg_pkg__msg__Sensor__rosidl_typesupport_introspection_c__get_function__Sensor__pose(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void snumsg_pkg__msg__Sensor__rosidl_typesupport_introspection_c__fetch_function__Sensor__pose(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    snumsg_pkg__msg__Sensor__rosidl_typesupport_introspection_c__get_const_function__Sensor__pose(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void snumsg_pkg__msg__Sensor__rosidl_typesupport_introspection_c__assign_function__Sensor__pose(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    snumsg_pkg__msg__Sensor__rosidl_typesupport_introspection_c__get_function__Sensor__pose(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

bool snumsg_pkg__msg__Sensor__rosidl_typesupport_introspection_c__resize_function__Sensor__pose(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  rosidl_runtime_c__float__Sequence__fini(member);
  return rosidl_runtime_c__float__Sequence__init(member, size);
}

size_t snumsg_pkg__msg__Sensor__rosidl_typesupport_introspection_c__size_function__Sensor__vel(
  const void * untyped_member)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return member->size;
}

const void * snumsg_pkg__msg__Sensor__rosidl_typesupport_introspection_c__get_const_function__Sensor__vel(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void * snumsg_pkg__msg__Sensor__rosidl_typesupport_introspection_c__get_function__Sensor__vel(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void snumsg_pkg__msg__Sensor__rosidl_typesupport_introspection_c__fetch_function__Sensor__vel(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    snumsg_pkg__msg__Sensor__rosidl_typesupport_introspection_c__get_const_function__Sensor__vel(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void snumsg_pkg__msg__Sensor__rosidl_typesupport_introspection_c__assign_function__Sensor__vel(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    snumsg_pkg__msg__Sensor__rosidl_typesupport_introspection_c__get_function__Sensor__vel(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

bool snumsg_pkg__msg__Sensor__rosidl_typesupport_introspection_c__resize_function__Sensor__vel(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  rosidl_runtime_c__float__Sequence__fini(member);
  return rosidl_runtime_c__float__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember snumsg_pkg__msg__Sensor__rosidl_typesupport_introspection_c__Sensor_message_member_array[4] = {
  {
    "tick",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(snumsg_pkg__msg__Sensor, tick),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "status",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(snumsg_pkg__msg__Sensor, status),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "pose",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(snumsg_pkg__msg__Sensor, pose),  // bytes offset in struct
    NULL,  // default value
    snumsg_pkg__msg__Sensor__rosidl_typesupport_introspection_c__size_function__Sensor__pose,  // size() function pointer
    snumsg_pkg__msg__Sensor__rosidl_typesupport_introspection_c__get_const_function__Sensor__pose,  // get_const(index) function pointer
    snumsg_pkg__msg__Sensor__rosidl_typesupport_introspection_c__get_function__Sensor__pose,  // get(index) function pointer
    snumsg_pkg__msg__Sensor__rosidl_typesupport_introspection_c__fetch_function__Sensor__pose,  // fetch(index, &value) function pointer
    snumsg_pkg__msg__Sensor__rosidl_typesupport_introspection_c__assign_function__Sensor__pose,  // assign(index, value) function pointer
    snumsg_pkg__msg__Sensor__rosidl_typesupport_introspection_c__resize_function__Sensor__pose  // resize(index) function pointer
  },
  {
    "vel",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(snumsg_pkg__msg__Sensor, vel),  // bytes offset in struct
    NULL,  // default value
    snumsg_pkg__msg__Sensor__rosidl_typesupport_introspection_c__size_function__Sensor__vel,  // size() function pointer
    snumsg_pkg__msg__Sensor__rosidl_typesupport_introspection_c__get_const_function__Sensor__vel,  // get_const(index) function pointer
    snumsg_pkg__msg__Sensor__rosidl_typesupport_introspection_c__get_function__Sensor__vel,  // get(index) function pointer
    snumsg_pkg__msg__Sensor__rosidl_typesupport_introspection_c__fetch_function__Sensor__vel,  // fetch(index, &value) function pointer
    snumsg_pkg__msg__Sensor__rosidl_typesupport_introspection_c__assign_function__Sensor__vel,  // assign(index, value) function pointer
    snumsg_pkg__msg__Sensor__rosidl_typesupport_introspection_c__resize_function__Sensor__vel  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers snumsg_pkg__msg__Sensor__rosidl_typesupport_introspection_c__Sensor_message_members = {
  "snumsg_pkg__msg",  // message namespace
  "Sensor",  // message name
  4,  // number of fields
  sizeof(snumsg_pkg__msg__Sensor),
  snumsg_pkg__msg__Sensor__rosidl_typesupport_introspection_c__Sensor_message_member_array,  // message members
  snumsg_pkg__msg__Sensor__rosidl_typesupport_introspection_c__Sensor_init_function,  // function to initialize message memory (memory has to be allocated)
  snumsg_pkg__msg__Sensor__rosidl_typesupport_introspection_c__Sensor_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t snumsg_pkg__msg__Sensor__rosidl_typesupport_introspection_c__Sensor_message_type_support_handle = {
  0,
  &snumsg_pkg__msg__Sensor__rosidl_typesupport_introspection_c__Sensor_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_snumsg_pkg
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, snumsg_pkg, msg, Sensor)() {
  snumsg_pkg__msg__Sensor__rosidl_typesupport_introspection_c__Sensor_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, builtin_interfaces, msg, Time)();
  if (!snumsg_pkg__msg__Sensor__rosidl_typesupport_introspection_c__Sensor_message_type_support_handle.typesupport_identifier) {
    snumsg_pkg__msg__Sensor__rosidl_typesupport_introspection_c__Sensor_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &snumsg_pkg__msg__Sensor__rosidl_typesupport_introspection_c__Sensor_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
