// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from snumsg_pkg:msg/MissionCode.idl
// generated code does not contain a copyright notice
#include "snumsg_pkg/msg/detail/mission_code__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `tick`
#include "builtin_interfaces/msg/detail/time__functions.h"
// Member `mission_code`
#include "rosidl_runtime_c/string_functions.h"

bool
snumsg_pkg__msg__MissionCode__init(snumsg_pkg__msg__MissionCode * msg)
{
  if (!msg) {
    return false;
  }
  // tick
  if (!builtin_interfaces__msg__Time__init(&msg->tick)) {
    snumsg_pkg__msg__MissionCode__fini(msg);
    return false;
  }
  // mission_code
  if (!rosidl_runtime_c__String__init(&msg->mission_code)) {
    snumsg_pkg__msg__MissionCode__fini(msg);
    return false;
  }
  return true;
}

void
snumsg_pkg__msg__MissionCode__fini(snumsg_pkg__msg__MissionCode * msg)
{
  if (!msg) {
    return;
  }
  // tick
  builtin_interfaces__msg__Time__fini(&msg->tick);
  // mission_code
  rosidl_runtime_c__String__fini(&msg->mission_code);
}

bool
snumsg_pkg__msg__MissionCode__are_equal(const snumsg_pkg__msg__MissionCode * lhs, const snumsg_pkg__msg__MissionCode * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // tick
  if (!builtin_interfaces__msg__Time__are_equal(
      &(lhs->tick), &(rhs->tick)))
  {
    return false;
  }
  // mission_code
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->mission_code), &(rhs->mission_code)))
  {
    return false;
  }
  return true;
}

bool
snumsg_pkg__msg__MissionCode__copy(
  const snumsg_pkg__msg__MissionCode * input,
  snumsg_pkg__msg__MissionCode * output)
{
  if (!input || !output) {
    return false;
  }
  // tick
  if (!builtin_interfaces__msg__Time__copy(
      &(input->tick), &(output->tick)))
  {
    return false;
  }
  // mission_code
  if (!rosidl_runtime_c__String__copy(
      &(input->mission_code), &(output->mission_code)))
  {
    return false;
  }
  return true;
}

snumsg_pkg__msg__MissionCode *
snumsg_pkg__msg__MissionCode__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  snumsg_pkg__msg__MissionCode * msg = (snumsg_pkg__msg__MissionCode *)allocator.allocate(sizeof(snumsg_pkg__msg__MissionCode), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(snumsg_pkg__msg__MissionCode));
  bool success = snumsg_pkg__msg__MissionCode__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
snumsg_pkg__msg__MissionCode__destroy(snumsg_pkg__msg__MissionCode * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    snumsg_pkg__msg__MissionCode__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
snumsg_pkg__msg__MissionCode__Sequence__init(snumsg_pkg__msg__MissionCode__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  snumsg_pkg__msg__MissionCode * data = NULL;

  if (size) {
    data = (snumsg_pkg__msg__MissionCode *)allocator.zero_allocate(size, sizeof(snumsg_pkg__msg__MissionCode), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = snumsg_pkg__msg__MissionCode__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        snumsg_pkg__msg__MissionCode__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
snumsg_pkg__msg__MissionCode__Sequence__fini(snumsg_pkg__msg__MissionCode__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      snumsg_pkg__msg__MissionCode__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

snumsg_pkg__msg__MissionCode__Sequence *
snumsg_pkg__msg__MissionCode__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  snumsg_pkg__msg__MissionCode__Sequence * array = (snumsg_pkg__msg__MissionCode__Sequence *)allocator.allocate(sizeof(snumsg_pkg__msg__MissionCode__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = snumsg_pkg__msg__MissionCode__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
snumsg_pkg__msg__MissionCode__Sequence__destroy(snumsg_pkg__msg__MissionCode__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    snumsg_pkg__msg__MissionCode__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
snumsg_pkg__msg__MissionCode__Sequence__are_equal(const snumsg_pkg__msg__MissionCode__Sequence * lhs, const snumsg_pkg__msg__MissionCode__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!snumsg_pkg__msg__MissionCode__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
snumsg_pkg__msg__MissionCode__Sequence__copy(
  const snumsg_pkg__msg__MissionCode__Sequence * input,
  snumsg_pkg__msg__MissionCode__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(snumsg_pkg__msg__MissionCode);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    snumsg_pkg__msg__MissionCode * data =
      (snumsg_pkg__msg__MissionCode *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!snumsg_pkg__msg__MissionCode__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          snumsg_pkg__msg__MissionCode__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!snumsg_pkg__msg__MissionCode__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
