// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from robotiq_2f_gripper_msgs:action/MoveTwoFingerGripper.idl
// generated code does not contain a copyright notice
#include "robotiq_2f_gripper_msgs/action/detail/move_two_finger_gripper__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Goal__init(robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Goal * msg)
{
  if (!msg) {
    return false;
  }
  // target_position
  // target_speed
  msg->target_speed = 1.0f;
  // target_force
  msg->target_force = 0.5f;
  return true;
}

void
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Goal__fini(robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Goal * msg)
{
  if (!msg) {
    return;
  }
  // target_position
  // target_speed
  // target_force
}

bool
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Goal__are_equal(const robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Goal * lhs, const robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Goal * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // target_position
  if (lhs->target_position != rhs->target_position) {
    return false;
  }
  // target_speed
  if (lhs->target_speed != rhs->target_speed) {
    return false;
  }
  // target_force
  if (lhs->target_force != rhs->target_force) {
    return false;
  }
  return true;
}

bool
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Goal__copy(
  const robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Goal * input,
  robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Goal * output)
{
  if (!input || !output) {
    return false;
  }
  // target_position
  output->target_position = input->target_position;
  // target_speed
  output->target_speed = input->target_speed;
  // target_force
  output->target_force = input->target_force;
  return true;
}

robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Goal *
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Goal__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Goal * msg = (robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Goal *)allocator.allocate(sizeof(robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Goal), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Goal));
  bool success = robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Goal__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Goal__destroy(robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Goal * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Goal__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Goal__Sequence__init(robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Goal__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Goal * data = NULL;

  if (size) {
    data = (robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Goal *)allocator.zero_allocate(size, sizeof(robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Goal), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Goal__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Goal__fini(&data[i - 1]);
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
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Goal__Sequence__fini(robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Goal__Sequence * array)
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
      robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Goal__fini(&array->data[i]);
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

robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Goal__Sequence *
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Goal__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Goal__Sequence * array = (robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Goal__Sequence *)allocator.allocate(sizeof(robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Goal__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Goal__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Goal__Sequence__destroy(robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Goal__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Goal__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Goal__Sequence__are_equal(const robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Goal__Sequence * lhs, const robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Goal__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Goal__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Goal__Sequence__copy(
  const robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Goal__Sequence * input,
  robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Goal__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Goal);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Goal * data =
      (robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Goal *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Goal__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Goal__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Goal__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Result__init(robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Result * msg)
{
  if (!msg) {
    return false;
  }
  // success
  return true;
}

void
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Result__fini(robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Result * msg)
{
  if (!msg) {
    return;
  }
  // success
}

bool
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Result__are_equal(const robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Result * lhs, const robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Result * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  return true;
}

bool
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Result__copy(
  const robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Result * input,
  robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Result * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  return true;
}

robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Result *
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Result__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Result * msg = (robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Result *)allocator.allocate(sizeof(robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Result), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Result));
  bool success = robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Result__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Result__destroy(robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Result * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Result__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Result__Sequence__init(robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Result__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Result * data = NULL;

  if (size) {
    data = (robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Result *)allocator.zero_allocate(size, sizeof(robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Result), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Result__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Result__fini(&data[i - 1]);
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
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Result__Sequence__fini(robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Result__Sequence * array)
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
      robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Result__fini(&array->data[i]);
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

robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Result__Sequence *
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Result__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Result__Sequence * array = (robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Result__Sequence *)allocator.allocate(sizeof(robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Result__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Result__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Result__Sequence__destroy(robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Result__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Result__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Result__Sequence__are_equal(const robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Result__Sequence * lhs, const robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Result__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Result__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Result__Sequence__copy(
  const robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Result__Sequence * input,
  robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Result__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Result);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Result * data =
      (robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Result *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Result__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Result__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Result__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `feedback`
#include "rosidl_runtime_c/string_functions.h"

bool
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Feedback__init(robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Feedback * msg)
{
  if (!msg) {
    return false;
  }
  // feedback
  if (!rosidl_runtime_c__String__init(&msg->feedback)) {
    robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Feedback__fini(msg);
    return false;
  }
  return true;
}

void
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Feedback__fini(robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Feedback * msg)
{
  if (!msg) {
    return;
  }
  // feedback
  rosidl_runtime_c__String__fini(&msg->feedback);
}

bool
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Feedback__are_equal(const robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Feedback * lhs, const robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Feedback * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // feedback
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->feedback), &(rhs->feedback)))
  {
    return false;
  }
  return true;
}

bool
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Feedback__copy(
  const robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Feedback * input,
  robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Feedback * output)
{
  if (!input || !output) {
    return false;
  }
  // feedback
  if (!rosidl_runtime_c__String__copy(
      &(input->feedback), &(output->feedback)))
  {
    return false;
  }
  return true;
}

robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Feedback *
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Feedback__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Feedback * msg = (robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Feedback *)allocator.allocate(sizeof(robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Feedback), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Feedback));
  bool success = robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Feedback__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Feedback__destroy(robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Feedback * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Feedback__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Feedback__Sequence__init(robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Feedback__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Feedback * data = NULL;

  if (size) {
    data = (robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Feedback *)allocator.zero_allocate(size, sizeof(robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Feedback), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Feedback__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Feedback__fini(&data[i - 1]);
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
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Feedback__Sequence__fini(robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Feedback__Sequence * array)
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
      robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Feedback__fini(&array->data[i]);
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

robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Feedback__Sequence *
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Feedback__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Feedback__Sequence * array = (robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Feedback__Sequence *)allocator.allocate(sizeof(robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Feedback__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Feedback__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Feedback__Sequence__destroy(robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Feedback__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Feedback__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Feedback__Sequence__are_equal(const robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Feedback__Sequence * lhs, const robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Feedback__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Feedback__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Feedback__Sequence__copy(
  const robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Feedback__Sequence * input,
  robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Feedback__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Feedback);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Feedback * data =
      (robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Feedback *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Feedback__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Feedback__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Feedback__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
#include "unique_identifier_msgs/msg/detail/uuid__functions.h"
// Member `goal`
// already included above
// #include "robotiq_2f_gripper_msgs/action/detail/move_two_finger_gripper__functions.h"

bool
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Request__init(robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Request * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Request__fini(msg);
    return false;
  }
  // goal
  if (!robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Goal__init(&msg->goal)) {
    robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Request__fini(msg);
    return false;
  }
  return true;
}

void
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Request__fini(robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Request * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
  // goal
  robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Goal__fini(&msg->goal);
}

bool
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Request__are_equal(const robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Request * lhs, const robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  // goal
  if (!robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Goal__are_equal(
      &(lhs->goal), &(rhs->goal)))
  {
    return false;
  }
  return true;
}

bool
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Request__copy(
  const robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Request * input,
  robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  // goal
  if (!robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Goal__copy(
      &(input->goal), &(output->goal)))
  {
    return false;
  }
  return true;
}

robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Request *
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Request__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Request * msg = (robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Request *)allocator.allocate(sizeof(robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Request));
  bool success = robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Request__destroy(robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Request__Sequence__init(robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Request * data = NULL;

  if (size) {
    data = (robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Request *)allocator.zero_allocate(size, sizeof(robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Request__fini(&data[i - 1]);
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
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Request__Sequence__fini(robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Request__Sequence * array)
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
      robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Request__fini(&array->data[i]);
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

robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Request__Sequence *
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Request__Sequence * array = (robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Request__Sequence *)allocator.allocate(sizeof(robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Request__Sequence__destroy(robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Request__Sequence__are_equal(const robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Request__Sequence * lhs, const robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Request__Sequence__copy(
  const robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Request__Sequence * input,
  robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Request * data =
      (robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `stamp`
#include "builtin_interfaces/msg/detail/time__functions.h"

bool
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Response__init(robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Response * msg)
{
  if (!msg) {
    return false;
  }
  // accepted
  // stamp
  if (!builtin_interfaces__msg__Time__init(&msg->stamp)) {
    robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Response__fini(msg);
    return false;
  }
  return true;
}

void
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Response__fini(robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Response * msg)
{
  if (!msg) {
    return;
  }
  // accepted
  // stamp
  builtin_interfaces__msg__Time__fini(&msg->stamp);
}

bool
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Response__are_equal(const robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Response * lhs, const robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // accepted
  if (lhs->accepted != rhs->accepted) {
    return false;
  }
  // stamp
  if (!builtin_interfaces__msg__Time__are_equal(
      &(lhs->stamp), &(rhs->stamp)))
  {
    return false;
  }
  return true;
}

bool
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Response__copy(
  const robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Response * input,
  robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // accepted
  output->accepted = input->accepted;
  // stamp
  if (!builtin_interfaces__msg__Time__copy(
      &(input->stamp), &(output->stamp)))
  {
    return false;
  }
  return true;
}

robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Response *
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Response__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Response * msg = (robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Response *)allocator.allocate(sizeof(robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Response));
  bool success = robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Response__destroy(robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Response__Sequence__init(robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Response * data = NULL;

  if (size) {
    data = (robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Response *)allocator.zero_allocate(size, sizeof(robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Response__fini(&data[i - 1]);
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
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Response__Sequence__fini(robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Response__Sequence * array)
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
      robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Response__fini(&array->data[i]);
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

robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Response__Sequence *
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Response__Sequence * array = (robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Response__Sequence *)allocator.allocate(sizeof(robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Response__Sequence__destroy(robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Response__Sequence__are_equal(const robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Response__Sequence * lhs, const robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Response__Sequence__copy(
  const robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Response__Sequence * input,
  robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Response * data =
      (robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `info`
#include "service_msgs/msg/detail/service_event_info__functions.h"
// Member `request`
// Member `response`
// already included above
// #include "robotiq_2f_gripper_msgs/action/detail/move_two_finger_gripper__functions.h"

bool
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Event__init(robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Event * msg)
{
  if (!msg) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__init(&msg->info)) {
    robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Event__fini(msg);
    return false;
  }
  // request
  if (!robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Request__Sequence__init(&msg->request, 0)) {
    robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Event__fini(msg);
    return false;
  }
  // response
  if (!robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Response__Sequence__init(&msg->response, 0)) {
    robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Event__fini(msg);
    return false;
  }
  return true;
}

void
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Event__fini(robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Event * msg)
{
  if (!msg) {
    return;
  }
  // info
  service_msgs__msg__ServiceEventInfo__fini(&msg->info);
  // request
  robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Request__Sequence__fini(&msg->request);
  // response
  robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Response__Sequence__fini(&msg->response);
}

bool
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Event__are_equal(const robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Event * lhs, const robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Event * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__are_equal(
      &(lhs->info), &(rhs->info)))
  {
    return false;
  }
  // request
  if (!robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Request__Sequence__are_equal(
      &(lhs->request), &(rhs->request)))
  {
    return false;
  }
  // response
  if (!robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Response__Sequence__are_equal(
      &(lhs->response), &(rhs->response)))
  {
    return false;
  }
  return true;
}

bool
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Event__copy(
  const robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Event * input,
  robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Event * output)
{
  if (!input || !output) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__copy(
      &(input->info), &(output->info)))
  {
    return false;
  }
  // request
  if (!robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Request__Sequence__copy(
      &(input->request), &(output->request)))
  {
    return false;
  }
  // response
  if (!robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Response__Sequence__copy(
      &(input->response), &(output->response)))
  {
    return false;
  }
  return true;
}

robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Event *
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Event__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Event * msg = (robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Event *)allocator.allocate(sizeof(robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Event), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Event));
  bool success = robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Event__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Event__destroy(robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Event * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Event__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Event__Sequence__init(robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Event__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Event * data = NULL;

  if (size) {
    data = (robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Event *)allocator.zero_allocate(size, sizeof(robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Event), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Event__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Event__fini(&data[i - 1]);
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
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Event__Sequence__fini(robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Event__Sequence * array)
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
      robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Event__fini(&array->data[i]);
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

robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Event__Sequence *
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Event__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Event__Sequence * array = (robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Event__Sequence *)allocator.allocate(sizeof(robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Event__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Event__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Event__Sequence__destroy(robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Event__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Event__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Event__Sequence__are_equal(const robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Event__Sequence * lhs, const robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Event__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Event__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Event__Sequence__copy(
  const robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Event__Sequence * input,
  robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Event__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Event);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Event * data =
      (robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Event *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Event__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Event__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Event__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__functions.h"

bool
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Request__init(robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Request * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Request__fini(msg);
    return false;
  }
  return true;
}

void
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Request__fini(robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Request * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
}

bool
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Request__are_equal(const robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Request * lhs, const robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  return true;
}

bool
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Request__copy(
  const robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Request * input,
  robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  return true;
}

robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Request *
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Request__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Request * msg = (robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Request *)allocator.allocate(sizeof(robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Request));
  bool success = robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Request__destroy(robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Request__Sequence__init(robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Request * data = NULL;

  if (size) {
    data = (robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Request *)allocator.zero_allocate(size, sizeof(robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Request__fini(&data[i - 1]);
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
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Request__Sequence__fini(robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Request__Sequence * array)
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
      robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Request__fini(&array->data[i]);
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

robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Request__Sequence *
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Request__Sequence * array = (robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Request__Sequence *)allocator.allocate(sizeof(robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Request__Sequence__destroy(robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Request__Sequence__are_equal(const robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Request__Sequence * lhs, const robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Request__Sequence__copy(
  const robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Request__Sequence * input,
  robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Request * data =
      (robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `result`
// already included above
// #include "robotiq_2f_gripper_msgs/action/detail/move_two_finger_gripper__functions.h"

bool
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Response__init(robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Response * msg)
{
  if (!msg) {
    return false;
  }
  // status
  // result
  if (!robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Result__init(&msg->result)) {
    robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Response__fini(msg);
    return false;
  }
  return true;
}

void
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Response__fini(robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Response * msg)
{
  if (!msg) {
    return;
  }
  // status
  // result
  robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Result__fini(&msg->result);
}

bool
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Response__are_equal(const robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Response * lhs, const robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // status
  if (lhs->status != rhs->status) {
    return false;
  }
  // result
  if (!robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Result__are_equal(
      &(lhs->result), &(rhs->result)))
  {
    return false;
  }
  return true;
}

bool
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Response__copy(
  const robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Response * input,
  robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // status
  output->status = input->status;
  // result
  if (!robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Result__copy(
      &(input->result), &(output->result)))
  {
    return false;
  }
  return true;
}

robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Response *
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Response__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Response * msg = (robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Response *)allocator.allocate(sizeof(robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Response));
  bool success = robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Response__destroy(robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Response__Sequence__init(robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Response * data = NULL;

  if (size) {
    data = (robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Response *)allocator.zero_allocate(size, sizeof(robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Response__fini(&data[i - 1]);
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
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Response__Sequence__fini(robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Response__Sequence * array)
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
      robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Response__fini(&array->data[i]);
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

robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Response__Sequence *
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Response__Sequence * array = (robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Response__Sequence *)allocator.allocate(sizeof(robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Response__Sequence__destroy(robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Response__Sequence__are_equal(const robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Response__Sequence * lhs, const robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Response__Sequence__copy(
  const robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Response__Sequence * input,
  robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Response * data =
      (robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `info`
// already included above
// #include "service_msgs/msg/detail/service_event_info__functions.h"
// Member `request`
// Member `response`
// already included above
// #include "robotiq_2f_gripper_msgs/action/detail/move_two_finger_gripper__functions.h"

bool
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Event__init(robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Event * msg)
{
  if (!msg) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__init(&msg->info)) {
    robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Event__fini(msg);
    return false;
  }
  // request
  if (!robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Request__Sequence__init(&msg->request, 0)) {
    robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Event__fini(msg);
    return false;
  }
  // response
  if (!robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Response__Sequence__init(&msg->response, 0)) {
    robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Event__fini(msg);
    return false;
  }
  return true;
}

void
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Event__fini(robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Event * msg)
{
  if (!msg) {
    return;
  }
  // info
  service_msgs__msg__ServiceEventInfo__fini(&msg->info);
  // request
  robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Request__Sequence__fini(&msg->request);
  // response
  robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Response__Sequence__fini(&msg->response);
}

bool
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Event__are_equal(const robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Event * lhs, const robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Event * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__are_equal(
      &(lhs->info), &(rhs->info)))
  {
    return false;
  }
  // request
  if (!robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Request__Sequence__are_equal(
      &(lhs->request), &(rhs->request)))
  {
    return false;
  }
  // response
  if (!robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Response__Sequence__are_equal(
      &(lhs->response), &(rhs->response)))
  {
    return false;
  }
  return true;
}

bool
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Event__copy(
  const robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Event * input,
  robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Event * output)
{
  if (!input || !output) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__copy(
      &(input->info), &(output->info)))
  {
    return false;
  }
  // request
  if (!robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Request__Sequence__copy(
      &(input->request), &(output->request)))
  {
    return false;
  }
  // response
  if (!robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Response__Sequence__copy(
      &(input->response), &(output->response)))
  {
    return false;
  }
  return true;
}

robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Event *
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Event__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Event * msg = (robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Event *)allocator.allocate(sizeof(robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Event), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Event));
  bool success = robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Event__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Event__destroy(robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Event * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Event__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Event__Sequence__init(robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Event__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Event * data = NULL;

  if (size) {
    data = (robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Event *)allocator.zero_allocate(size, sizeof(robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Event), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Event__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Event__fini(&data[i - 1]);
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
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Event__Sequence__fini(robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Event__Sequence * array)
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
      robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Event__fini(&array->data[i]);
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

robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Event__Sequence *
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Event__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Event__Sequence * array = (robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Event__Sequence *)allocator.allocate(sizeof(robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Event__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Event__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Event__Sequence__destroy(robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Event__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Event__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Event__Sequence__are_equal(const robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Event__Sequence * lhs, const robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Event__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Event__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Event__Sequence__copy(
  const robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Event__Sequence * input,
  robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Event__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Event);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Event * data =
      (robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Event *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Event__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Event__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Event__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__functions.h"
// Member `feedback`
// already included above
// #include "robotiq_2f_gripper_msgs/action/detail/move_two_finger_gripper__functions.h"

bool
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_FeedbackMessage__init(robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_FeedbackMessage * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_FeedbackMessage__fini(msg);
    return false;
  }
  // feedback
  if (!robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Feedback__init(&msg->feedback)) {
    robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_FeedbackMessage__fini(msg);
    return false;
  }
  return true;
}

void
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_FeedbackMessage__fini(robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_FeedbackMessage * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
  // feedback
  robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Feedback__fini(&msg->feedback);
}

bool
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_FeedbackMessage__are_equal(const robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_FeedbackMessage * lhs, const robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_FeedbackMessage * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  // feedback
  if (!robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Feedback__are_equal(
      &(lhs->feedback), &(rhs->feedback)))
  {
    return false;
  }
  return true;
}

bool
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_FeedbackMessage__copy(
  const robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_FeedbackMessage * input,
  robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_FeedbackMessage * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  // feedback
  if (!robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Feedback__copy(
      &(input->feedback), &(output->feedback)))
  {
    return false;
  }
  return true;
}

robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_FeedbackMessage *
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_FeedbackMessage__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_FeedbackMessage * msg = (robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_FeedbackMessage *)allocator.allocate(sizeof(robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_FeedbackMessage), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_FeedbackMessage));
  bool success = robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_FeedbackMessage__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_FeedbackMessage__destroy(robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_FeedbackMessage * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_FeedbackMessage__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_FeedbackMessage__Sequence__init(robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_FeedbackMessage__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_FeedbackMessage * data = NULL;

  if (size) {
    data = (robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_FeedbackMessage *)allocator.zero_allocate(size, sizeof(robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_FeedbackMessage), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_FeedbackMessage__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_FeedbackMessage__fini(&data[i - 1]);
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
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_FeedbackMessage__Sequence__fini(robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_FeedbackMessage__Sequence * array)
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
      robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_FeedbackMessage__fini(&array->data[i]);
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

robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_FeedbackMessage__Sequence *
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_FeedbackMessage__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_FeedbackMessage__Sequence * array = (robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_FeedbackMessage__Sequence *)allocator.allocate(sizeof(robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_FeedbackMessage__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_FeedbackMessage__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_FeedbackMessage__Sequence__destroy(robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_FeedbackMessage__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_FeedbackMessage__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_FeedbackMessage__Sequence__are_equal(const robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_FeedbackMessage__Sequence * lhs, const robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_FeedbackMessage__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_FeedbackMessage__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_FeedbackMessage__Sequence__copy(
  const robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_FeedbackMessage__Sequence * input,
  robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_FeedbackMessage__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_FeedbackMessage);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_FeedbackMessage * data =
      (robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_FeedbackMessage *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_FeedbackMessage__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_FeedbackMessage__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_FeedbackMessage__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
