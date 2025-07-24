// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from snumsg_pkg:msg/MissionCode.idl
// generated code does not contain a copyright notice

#ifndef SNUMSG_PKG__MSG__DETAIL__MISSION_CODE__FUNCTIONS_H_
#define SNUMSG_PKG__MSG__DETAIL__MISSION_CODE__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "snumsg_pkg/msg/rosidl_generator_c__visibility_control.h"

#include "snumsg_pkg/msg/detail/mission_code__struct.h"

/// Initialize msg/MissionCode message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * snumsg_pkg__msg__MissionCode
 * )) before or use
 * snumsg_pkg__msg__MissionCode__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_snumsg_pkg
bool
snumsg_pkg__msg__MissionCode__init(snumsg_pkg__msg__MissionCode * msg);

/// Finalize msg/MissionCode message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_snumsg_pkg
void
snumsg_pkg__msg__MissionCode__fini(snumsg_pkg__msg__MissionCode * msg);

/// Create msg/MissionCode message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * snumsg_pkg__msg__MissionCode__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_snumsg_pkg
snumsg_pkg__msg__MissionCode *
snumsg_pkg__msg__MissionCode__create();

/// Destroy msg/MissionCode message.
/**
 * It calls
 * snumsg_pkg__msg__MissionCode__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_snumsg_pkg
void
snumsg_pkg__msg__MissionCode__destroy(snumsg_pkg__msg__MissionCode * msg);

/// Check for msg/MissionCode message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_snumsg_pkg
bool
snumsg_pkg__msg__MissionCode__are_equal(const snumsg_pkg__msg__MissionCode * lhs, const snumsg_pkg__msg__MissionCode * rhs);

/// Copy a msg/MissionCode message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_snumsg_pkg
bool
snumsg_pkg__msg__MissionCode__copy(
  const snumsg_pkg__msg__MissionCode * input,
  snumsg_pkg__msg__MissionCode * output);

/// Initialize array of msg/MissionCode messages.
/**
 * It allocates the memory for the number of elements and calls
 * snumsg_pkg__msg__MissionCode__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_snumsg_pkg
bool
snumsg_pkg__msg__MissionCode__Sequence__init(snumsg_pkg__msg__MissionCode__Sequence * array, size_t size);

/// Finalize array of msg/MissionCode messages.
/**
 * It calls
 * snumsg_pkg__msg__MissionCode__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_snumsg_pkg
void
snumsg_pkg__msg__MissionCode__Sequence__fini(snumsg_pkg__msg__MissionCode__Sequence * array);

/// Create array of msg/MissionCode messages.
/**
 * It allocates the memory for the array and calls
 * snumsg_pkg__msg__MissionCode__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_snumsg_pkg
snumsg_pkg__msg__MissionCode__Sequence *
snumsg_pkg__msg__MissionCode__Sequence__create(size_t size);

/// Destroy array of msg/MissionCode messages.
/**
 * It calls
 * snumsg_pkg__msg__MissionCode__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_snumsg_pkg
void
snumsg_pkg__msg__MissionCode__Sequence__destroy(snumsg_pkg__msg__MissionCode__Sequence * array);

/// Check for msg/MissionCode message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_snumsg_pkg
bool
snumsg_pkg__msg__MissionCode__Sequence__are_equal(const snumsg_pkg__msg__MissionCode__Sequence * lhs, const snumsg_pkg__msg__MissionCode__Sequence * rhs);

/// Copy an array of msg/MissionCode messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_snumsg_pkg
bool
snumsg_pkg__msg__MissionCode__Sequence__copy(
  const snumsg_pkg__msg__MissionCode__Sequence * input,
  snumsg_pkg__msg__MissionCode__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // SNUMSG_PKG__MSG__DETAIL__MISSION_CODE__FUNCTIONS_H_
