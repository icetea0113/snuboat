// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from snumsg_pkg:msg/MissionCode.idl
// generated code does not contain a copyright notice

#ifndef SNUMSG_PKG__MSG__DETAIL__MISSION_CODE__STRUCT_H_
#define SNUMSG_PKG__MSG__DETAIL__MISSION_CODE__STRUCT_H_

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
// Member 'mission_code'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/MissionCode in the package snumsg_pkg.
typedef struct snumsg_pkg__msg__MissionCode
{
  builtin_interfaces__msg__Time tick;
  rosidl_runtime_c__String mission_code;
} snumsg_pkg__msg__MissionCode;

// Struct for a sequence of snumsg_pkg__msg__MissionCode.
typedef struct snumsg_pkg__msg__MissionCode__Sequence
{
  snumsg_pkg__msg__MissionCode * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} snumsg_pkg__msg__MissionCode__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SNUMSG_PKG__MSG__DETAIL__MISSION_CODE__STRUCT_H_
