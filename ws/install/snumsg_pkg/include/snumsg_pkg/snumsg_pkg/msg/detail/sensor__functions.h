// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from snumsg_pkg:msg/Sensor.idl
// generated code does not contain a copyright notice

#ifndef SNUMSG_PKG__MSG__DETAIL__SENSOR__FUNCTIONS_H_
#define SNUMSG_PKG__MSG__DETAIL__SENSOR__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "snumsg_pkg/msg/rosidl_generator_c__visibility_control.h"

#include "snumsg_pkg/msg/detail/sensor__struct.h"

/// Initialize msg/Sensor message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * snumsg_pkg__msg__Sensor
 * )) before or use
 * snumsg_pkg__msg__Sensor__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_snumsg_pkg
bool
snumsg_pkg__msg__Sensor__init(snumsg_pkg__msg__Sensor * msg);

/// Finalize msg/Sensor message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_snumsg_pkg
void
snumsg_pkg__msg__Sensor__fini(snumsg_pkg__msg__Sensor * msg);

/// Create msg/Sensor message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * snumsg_pkg__msg__Sensor__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_snumsg_pkg
snumsg_pkg__msg__Sensor *
snumsg_pkg__msg__Sensor__create();

/// Destroy msg/Sensor message.
/**
 * It calls
 * snumsg_pkg__msg__Sensor__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_snumsg_pkg
void
snumsg_pkg__msg__Sensor__destroy(snumsg_pkg__msg__Sensor * msg);

/// Check for msg/Sensor message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_snumsg_pkg
bool
snumsg_pkg__msg__Sensor__are_equal(const snumsg_pkg__msg__Sensor * lhs, const snumsg_pkg__msg__Sensor * rhs);

/// Copy a msg/Sensor message.
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
snumsg_pkg__msg__Sensor__copy(
  const snumsg_pkg__msg__Sensor * input,
  snumsg_pkg__msg__Sensor * output);

/// Initialize array of msg/Sensor messages.
/**
 * It allocates the memory for the number of elements and calls
 * snumsg_pkg__msg__Sensor__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_snumsg_pkg
bool
snumsg_pkg__msg__Sensor__Sequence__init(snumsg_pkg__msg__Sensor__Sequence * array, size_t size);

/// Finalize array of msg/Sensor messages.
/**
 * It calls
 * snumsg_pkg__msg__Sensor__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_snumsg_pkg
void
snumsg_pkg__msg__Sensor__Sequence__fini(snumsg_pkg__msg__Sensor__Sequence * array);

/// Create array of msg/Sensor messages.
/**
 * It allocates the memory for the array and calls
 * snumsg_pkg__msg__Sensor__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_snumsg_pkg
snumsg_pkg__msg__Sensor__Sequence *
snumsg_pkg__msg__Sensor__Sequence__create(size_t size);

/// Destroy array of msg/Sensor messages.
/**
 * It calls
 * snumsg_pkg__msg__Sensor__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_snumsg_pkg
void
snumsg_pkg__msg__Sensor__Sequence__destroy(snumsg_pkg__msg__Sensor__Sequence * array);

/// Check for msg/Sensor message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_snumsg_pkg
bool
snumsg_pkg__msg__Sensor__Sequence__are_equal(const snumsg_pkg__msg__Sensor__Sequence * lhs, const snumsg_pkg__msg__Sensor__Sequence * rhs);

/// Copy an array of msg/Sensor messages.
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
snumsg_pkg__msg__Sensor__Sequence__copy(
  const snumsg_pkg__msg__Sensor__Sequence * input,
  snumsg_pkg__msg__Sensor__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // SNUMSG_PKG__MSG__DETAIL__SENSOR__FUNCTIONS_H_
