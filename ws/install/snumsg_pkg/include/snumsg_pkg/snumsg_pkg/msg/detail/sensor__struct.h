// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from snumsg_pkg:msg/Sensor.idl
// generated code does not contain a copyright notice

#ifndef SNUMSG_PKG__MSG__DETAIL__SENSOR__STRUCT_H_
#define SNUMSG_PKG__MSG__DETAIL__SENSOR__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'tick'
#include "builtin_interfaces/msg/detail/time__struct.h"
// Member 'status'
#include "rosidl_runtime_c/string.h"
// Member 'pose'
// Member 'vel'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/Sensor in the package snumsg_pkg.
typedef struct snumsg_pkg__msg__Sensor
{
  builtin_interfaces__msg__Time tick;
  rosidl_runtime_c__String status;
  rosidl_runtime_c__float__Sequence pose;
  rosidl_runtime_c__float__Sequence vel;
} snumsg_pkg__msg__Sensor;

// Struct for a sequence of snumsg_pkg__msg__Sensor.
typedef struct snumsg_pkg__msg__Sensor__Sequence
{
  snumsg_pkg__msg__Sensor * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} snumsg_pkg__msg__Sensor__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SNUMSG_PKG__MSG__DETAIL__SENSOR__STRUCT_H_
