// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from snumsg_pkg:msg/Sensor.idl
// generated code does not contain a copyright notice
#include "snumsg_pkg/msg/detail/sensor__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `tick`
#include "builtin_interfaces/msg/detail/time__functions.h"
// Member `status`
#include "rosidl_runtime_c/string_functions.h"
// Member `pose`
// Member `vel`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
snumsg_pkg__msg__Sensor__init(snumsg_pkg__msg__Sensor * msg)
{
  if (!msg) {
    return false;
  }
  // tick
  if (!builtin_interfaces__msg__Time__init(&msg->tick)) {
    snumsg_pkg__msg__Sensor__fini(msg);
    return false;
  }
  // status
  if (!rosidl_runtime_c__String__init(&msg->status)) {
    snumsg_pkg__msg__Sensor__fini(msg);
    return false;
  }
  // pose
  if (!rosidl_runtime_c__float__Sequence__init(&msg->pose, 0)) {
    snumsg_pkg__msg__Sensor__fini(msg);
    return false;
  }
  // vel
  if (!rosidl_runtime_c__float__Sequence__init(&msg->vel, 0)) {
    snumsg_pkg__msg__Sensor__fini(msg);
    return false;
  }
  return true;
}

void
snumsg_pkg__msg__Sensor__fini(snumsg_pkg__msg__Sensor * msg)
{
  if (!msg) {
    return;
  }
  // tick
  builtin_interfaces__msg__Time__fini(&msg->tick);
  // status
  rosidl_runtime_c__String__fini(&msg->status);
  // pose
  rosidl_runtime_c__float__Sequence__fini(&msg->pose);
  // vel
  rosidl_runtime_c__float__Sequence__fini(&msg->vel);
}

bool
snumsg_pkg__msg__Sensor__are_equal(const snumsg_pkg__msg__Sensor * lhs, const snumsg_pkg__msg__Sensor * rhs)
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
  // status
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->status), &(rhs->status)))
  {
    return false;
  }
  // pose
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->pose), &(rhs->pose)))
  {
    return false;
  }
  // vel
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->vel), &(rhs->vel)))
  {
    return false;
  }
  return true;
}

bool
snumsg_pkg__msg__Sensor__copy(
  const snumsg_pkg__msg__Sensor * input,
  snumsg_pkg__msg__Sensor * output)
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
  // status
  if (!rosidl_runtime_c__String__copy(
      &(input->status), &(output->status)))
  {
    return false;
  }
  // pose
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->pose), &(output->pose)))
  {
    return false;
  }
  // vel
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->vel), &(output->vel)))
  {
    return false;
  }
  return true;
}

snumsg_pkg__msg__Sensor *
snumsg_pkg__msg__Sensor__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  snumsg_pkg__msg__Sensor * msg = (snumsg_pkg__msg__Sensor *)allocator.allocate(sizeof(snumsg_pkg__msg__Sensor), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(snumsg_pkg__msg__Sensor));
  bool success = snumsg_pkg__msg__Sensor__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
snumsg_pkg__msg__Sensor__destroy(snumsg_pkg__msg__Sensor * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    snumsg_pkg__msg__Sensor__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
snumsg_pkg__msg__Sensor__Sequence__init(snumsg_pkg__msg__Sensor__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  snumsg_pkg__msg__Sensor * data = NULL;

  if (size) {
    data = (snumsg_pkg__msg__Sensor *)allocator.zero_allocate(size, sizeof(snumsg_pkg__msg__Sensor), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = snumsg_pkg__msg__Sensor__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        snumsg_pkg__msg__Sensor__fini(&data[i - 1]);
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
snumsg_pkg__msg__Sensor__Sequence__fini(snumsg_pkg__msg__Sensor__Sequence * array)
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
      snumsg_pkg__msg__Sensor__fini(&array->data[i]);
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

snumsg_pkg__msg__Sensor__Sequence *
snumsg_pkg__msg__Sensor__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  snumsg_pkg__msg__Sensor__Sequence * array = (snumsg_pkg__msg__Sensor__Sequence *)allocator.allocate(sizeof(snumsg_pkg__msg__Sensor__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = snumsg_pkg__msg__Sensor__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
snumsg_pkg__msg__Sensor__Sequence__destroy(snumsg_pkg__msg__Sensor__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    snumsg_pkg__msg__Sensor__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
snumsg_pkg__msg__Sensor__Sequence__are_equal(const snumsg_pkg__msg__Sensor__Sequence * lhs, const snumsg_pkg__msg__Sensor__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!snumsg_pkg__msg__Sensor__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
snumsg_pkg__msg__Sensor__Sequence__copy(
  const snumsg_pkg__msg__Sensor__Sequence * input,
  snumsg_pkg__msg__Sensor__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(snumsg_pkg__msg__Sensor);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    snumsg_pkg__msg__Sensor * data =
      (snumsg_pkg__msg__Sensor *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!snumsg_pkg__msg__Sensor__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          snumsg_pkg__msg__Sensor__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!snumsg_pkg__msg__Sensor__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
