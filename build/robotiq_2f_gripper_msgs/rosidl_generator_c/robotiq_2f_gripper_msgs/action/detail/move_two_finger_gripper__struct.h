// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robotiq_2f_gripper_msgs:action/MoveTwoFingerGripper.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "robotiq_2f_gripper_msgs/action/move_two_finger_gripper.h"


#ifndef ROBOTIQ_2F_GRIPPER_MSGS__ACTION__DETAIL__MOVE_TWO_FINGER_GRIPPER__STRUCT_H_
#define ROBOTIQ_2F_GRIPPER_MSGS__ACTION__DETAIL__MOVE_TWO_FINGER_GRIPPER__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in action/MoveTwoFingerGripper in the package robotiq_2f_gripper_msgs.
typedef struct robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Goal
{
  /// target distance between fingers
  float target_position;
  /// [%] of max speed [0.0 - 1.0]
  float target_speed;
  /// [%] of max force [0.0 - 1.0]
  float target_force;
} robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Goal;

// Struct for a sequence of robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Goal.
typedef struct robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Goal__Sequence
{
  robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Goal__Sequence;

// Constants defined in the message

/// Struct defined in action/MoveTwoFingerGripper in the package robotiq_2f_gripper_msgs.
typedef struct robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Result
{
  bool success;
} robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Result;

// Struct for a sequence of robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Result.
typedef struct robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Result__Sequence
{
  robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Result__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'feedback'
#include "rosidl_runtime_c/string.h"

/// Struct defined in action/MoveTwoFingerGripper in the package robotiq_2f_gripper_msgs.
typedef struct robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Feedback
{
  rosidl_runtime_c__String feedback;
} robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Feedback;

// Struct for a sequence of robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Feedback.
typedef struct robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Feedback__Sequence
{
  robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Feedback__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "robotiq_2f_gripper_msgs/action/detail/move_two_finger_gripper__struct.h"

/// Struct defined in action/MoveTwoFingerGripper in the package robotiq_2f_gripper_msgs.
typedef struct robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Goal goal;
} robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Request;

// Struct for a sequence of robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Request.
typedef struct robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Request__Sequence
{
  robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in action/MoveTwoFingerGripper in the package robotiq_2f_gripper_msgs.
typedef struct robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Response;

// Struct for a sequence of robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Response.
typedef struct robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Response__Sequence
{
  robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Event__request__MAX_SIZE = 1
};
// response
enum
{
  robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Event__response__MAX_SIZE = 1
};

/// Struct defined in action/MoveTwoFingerGripper in the package robotiq_2f_gripper_msgs.
typedef struct robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Event
{
  service_msgs__msg__ServiceEventInfo info;
  robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Request__Sequence request;
  robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Response__Sequence response;
} robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Event;

// Struct for a sequence of robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Event.
typedef struct robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Event__Sequence
{
  robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_SendGoal_Event__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

/// Struct defined in action/MoveTwoFingerGripper in the package robotiq_2f_gripper_msgs.
typedef struct robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Request;

// Struct for a sequence of robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Request.
typedef struct robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Request__Sequence
{
  robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "robotiq_2f_gripper_msgs/action/detail/move_two_finger_gripper__struct.h"

/// Struct defined in action/MoveTwoFingerGripper in the package robotiq_2f_gripper_msgs.
typedef struct robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Response
{
  int8_t status;
  robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Result result;
} robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Response;

// Struct for a sequence of robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Response.
typedef struct robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Response__Sequence
{
  robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
// already included above
// #include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Event__request__MAX_SIZE = 1
};
// response
enum
{
  robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Event__response__MAX_SIZE = 1
};

/// Struct defined in action/MoveTwoFingerGripper in the package robotiq_2f_gripper_msgs.
typedef struct robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Event
{
  service_msgs__msg__ServiceEventInfo info;
  robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Request__Sequence request;
  robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Response__Sequence response;
} robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Event;

// Struct for a sequence of robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Event.
typedef struct robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Event__Sequence
{
  robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_GetResult_Event__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "robotiq_2f_gripper_msgs/action/detail/move_two_finger_gripper__struct.h"

/// Struct defined in action/MoveTwoFingerGripper in the package robotiq_2f_gripper_msgs.
typedef struct robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_Feedback feedback;
} robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_FeedbackMessage;

// Struct for a sequence of robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_FeedbackMessage.
typedef struct robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_FeedbackMessage__Sequence
{
  robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robotiq_2f_gripper_msgs__action__MoveTwoFingerGripper_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBOTIQ_2F_GRIPPER_MSGS__ACTION__DETAIL__MOVE_TWO_FINGER_GRIPPER__STRUCT_H_
