// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robotiq_2f_gripper_msgs:action/MoveTwoFingerGripper.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "robotiq_2f_gripper_msgs/action/move_two_finger_gripper.hpp"


#ifndef ROBOTIQ_2F_GRIPPER_MSGS__ACTION__DETAIL__MOVE_TWO_FINGER_GRIPPER__BUILDER_HPP_
#define ROBOTIQ_2F_GRIPPER_MSGS__ACTION__DETAIL__MOVE_TWO_FINGER_GRIPPER__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robotiq_2f_gripper_msgs/action/detail/move_two_finger_gripper__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robotiq_2f_gripper_msgs
{

namespace action
{

namespace builder
{

class Init_MoveTwoFingerGripper_Goal_target_force
{
public:
  explicit Init_MoveTwoFingerGripper_Goal_target_force(::robotiq_2f_gripper_msgs::action::MoveTwoFingerGripper_Goal & msg)
  : msg_(msg)
  {}
  ::robotiq_2f_gripper_msgs::action::MoveTwoFingerGripper_Goal target_force(::robotiq_2f_gripper_msgs::action::MoveTwoFingerGripper_Goal::_target_force_type arg)
  {
    msg_.target_force = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robotiq_2f_gripper_msgs::action::MoveTwoFingerGripper_Goal msg_;
};

class Init_MoveTwoFingerGripper_Goal_target_speed
{
public:
  explicit Init_MoveTwoFingerGripper_Goal_target_speed(::robotiq_2f_gripper_msgs::action::MoveTwoFingerGripper_Goal & msg)
  : msg_(msg)
  {}
  Init_MoveTwoFingerGripper_Goal_target_force target_speed(::robotiq_2f_gripper_msgs::action::MoveTwoFingerGripper_Goal::_target_speed_type arg)
  {
    msg_.target_speed = std::move(arg);
    return Init_MoveTwoFingerGripper_Goal_target_force(msg_);
  }

private:
  ::robotiq_2f_gripper_msgs::action::MoveTwoFingerGripper_Goal msg_;
};

class Init_MoveTwoFingerGripper_Goal_target_position
{
public:
  Init_MoveTwoFingerGripper_Goal_target_position()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MoveTwoFingerGripper_Goal_target_speed target_position(::robotiq_2f_gripper_msgs::action::MoveTwoFingerGripper_Goal::_target_position_type arg)
  {
    msg_.target_position = std::move(arg);
    return Init_MoveTwoFingerGripper_Goal_target_speed(msg_);
  }

private:
  ::robotiq_2f_gripper_msgs::action::MoveTwoFingerGripper_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::robotiq_2f_gripper_msgs::action::MoveTwoFingerGripper_Goal>()
{
  return robotiq_2f_gripper_msgs::action::builder::Init_MoveTwoFingerGripper_Goal_target_position();
}

}  // namespace robotiq_2f_gripper_msgs


namespace robotiq_2f_gripper_msgs
{

namespace action
{

namespace builder
{

class Init_MoveTwoFingerGripper_Result_success
{
public:
  Init_MoveTwoFingerGripper_Result_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::robotiq_2f_gripper_msgs::action::MoveTwoFingerGripper_Result success(::robotiq_2f_gripper_msgs::action::MoveTwoFingerGripper_Result::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robotiq_2f_gripper_msgs::action::MoveTwoFingerGripper_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::robotiq_2f_gripper_msgs::action::MoveTwoFingerGripper_Result>()
{
  return robotiq_2f_gripper_msgs::action::builder::Init_MoveTwoFingerGripper_Result_success();
}

}  // namespace robotiq_2f_gripper_msgs


namespace robotiq_2f_gripper_msgs
{

namespace action
{

namespace builder
{

class Init_MoveTwoFingerGripper_Feedback_feedback
{
public:
  Init_MoveTwoFingerGripper_Feedback_feedback()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::robotiq_2f_gripper_msgs::action::MoveTwoFingerGripper_Feedback feedback(::robotiq_2f_gripper_msgs::action::MoveTwoFingerGripper_Feedback::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robotiq_2f_gripper_msgs::action::MoveTwoFingerGripper_Feedback msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::robotiq_2f_gripper_msgs::action::MoveTwoFingerGripper_Feedback>()
{
  return robotiq_2f_gripper_msgs::action::builder::Init_MoveTwoFingerGripper_Feedback_feedback();
}

}  // namespace robotiq_2f_gripper_msgs


namespace robotiq_2f_gripper_msgs
{

namespace action
{

namespace builder
{

class Init_MoveTwoFingerGripper_SendGoal_Request_goal
{
public:
  explicit Init_MoveTwoFingerGripper_SendGoal_Request_goal(::robotiq_2f_gripper_msgs::action::MoveTwoFingerGripper_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::robotiq_2f_gripper_msgs::action::MoveTwoFingerGripper_SendGoal_Request goal(::robotiq_2f_gripper_msgs::action::MoveTwoFingerGripper_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robotiq_2f_gripper_msgs::action::MoveTwoFingerGripper_SendGoal_Request msg_;
};

class Init_MoveTwoFingerGripper_SendGoal_Request_goal_id
{
public:
  Init_MoveTwoFingerGripper_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MoveTwoFingerGripper_SendGoal_Request_goal goal_id(::robotiq_2f_gripper_msgs::action::MoveTwoFingerGripper_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_MoveTwoFingerGripper_SendGoal_Request_goal(msg_);
  }

private:
  ::robotiq_2f_gripper_msgs::action::MoveTwoFingerGripper_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::robotiq_2f_gripper_msgs::action::MoveTwoFingerGripper_SendGoal_Request>()
{
  return robotiq_2f_gripper_msgs::action::builder::Init_MoveTwoFingerGripper_SendGoal_Request_goal_id();
}

}  // namespace robotiq_2f_gripper_msgs


namespace robotiq_2f_gripper_msgs
{

namespace action
{

namespace builder
{

class Init_MoveTwoFingerGripper_SendGoal_Response_stamp
{
public:
  explicit Init_MoveTwoFingerGripper_SendGoal_Response_stamp(::robotiq_2f_gripper_msgs::action::MoveTwoFingerGripper_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::robotiq_2f_gripper_msgs::action::MoveTwoFingerGripper_SendGoal_Response stamp(::robotiq_2f_gripper_msgs::action::MoveTwoFingerGripper_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robotiq_2f_gripper_msgs::action::MoveTwoFingerGripper_SendGoal_Response msg_;
};

class Init_MoveTwoFingerGripper_SendGoal_Response_accepted
{
public:
  Init_MoveTwoFingerGripper_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MoveTwoFingerGripper_SendGoal_Response_stamp accepted(::robotiq_2f_gripper_msgs::action::MoveTwoFingerGripper_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_MoveTwoFingerGripper_SendGoal_Response_stamp(msg_);
  }

private:
  ::robotiq_2f_gripper_msgs::action::MoveTwoFingerGripper_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::robotiq_2f_gripper_msgs::action::MoveTwoFingerGripper_SendGoal_Response>()
{
  return robotiq_2f_gripper_msgs::action::builder::Init_MoveTwoFingerGripper_SendGoal_Response_accepted();
}

}  // namespace robotiq_2f_gripper_msgs


namespace robotiq_2f_gripper_msgs
{

namespace action
{

namespace builder
{

class Init_MoveTwoFingerGripper_SendGoal_Event_response
{
public:
  explicit Init_MoveTwoFingerGripper_SendGoal_Event_response(::robotiq_2f_gripper_msgs::action::MoveTwoFingerGripper_SendGoal_Event & msg)
  : msg_(msg)
  {}
  ::robotiq_2f_gripper_msgs::action::MoveTwoFingerGripper_SendGoal_Event response(::robotiq_2f_gripper_msgs::action::MoveTwoFingerGripper_SendGoal_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robotiq_2f_gripper_msgs::action::MoveTwoFingerGripper_SendGoal_Event msg_;
};

class Init_MoveTwoFingerGripper_SendGoal_Event_request
{
public:
  explicit Init_MoveTwoFingerGripper_SendGoal_Event_request(::robotiq_2f_gripper_msgs::action::MoveTwoFingerGripper_SendGoal_Event & msg)
  : msg_(msg)
  {}
  Init_MoveTwoFingerGripper_SendGoal_Event_response request(::robotiq_2f_gripper_msgs::action::MoveTwoFingerGripper_SendGoal_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_MoveTwoFingerGripper_SendGoal_Event_response(msg_);
  }

private:
  ::robotiq_2f_gripper_msgs::action::MoveTwoFingerGripper_SendGoal_Event msg_;
};

class Init_MoveTwoFingerGripper_SendGoal_Event_info
{
public:
  Init_MoveTwoFingerGripper_SendGoal_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MoveTwoFingerGripper_SendGoal_Event_request info(::robotiq_2f_gripper_msgs::action::MoveTwoFingerGripper_SendGoal_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_MoveTwoFingerGripper_SendGoal_Event_request(msg_);
  }

private:
  ::robotiq_2f_gripper_msgs::action::MoveTwoFingerGripper_SendGoal_Event msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::robotiq_2f_gripper_msgs::action::MoveTwoFingerGripper_SendGoal_Event>()
{
  return robotiq_2f_gripper_msgs::action::builder::Init_MoveTwoFingerGripper_SendGoal_Event_info();
}

}  // namespace robotiq_2f_gripper_msgs


namespace robotiq_2f_gripper_msgs
{

namespace action
{

namespace builder
{

class Init_MoveTwoFingerGripper_GetResult_Request_goal_id
{
public:
  Init_MoveTwoFingerGripper_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::robotiq_2f_gripper_msgs::action::MoveTwoFingerGripper_GetResult_Request goal_id(::robotiq_2f_gripper_msgs::action::MoveTwoFingerGripper_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robotiq_2f_gripper_msgs::action::MoveTwoFingerGripper_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::robotiq_2f_gripper_msgs::action::MoveTwoFingerGripper_GetResult_Request>()
{
  return robotiq_2f_gripper_msgs::action::builder::Init_MoveTwoFingerGripper_GetResult_Request_goal_id();
}

}  // namespace robotiq_2f_gripper_msgs


namespace robotiq_2f_gripper_msgs
{

namespace action
{

namespace builder
{

class Init_MoveTwoFingerGripper_GetResult_Response_result
{
public:
  explicit Init_MoveTwoFingerGripper_GetResult_Response_result(::robotiq_2f_gripper_msgs::action::MoveTwoFingerGripper_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::robotiq_2f_gripper_msgs::action::MoveTwoFingerGripper_GetResult_Response result(::robotiq_2f_gripper_msgs::action::MoveTwoFingerGripper_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robotiq_2f_gripper_msgs::action::MoveTwoFingerGripper_GetResult_Response msg_;
};

class Init_MoveTwoFingerGripper_GetResult_Response_status
{
public:
  Init_MoveTwoFingerGripper_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MoveTwoFingerGripper_GetResult_Response_result status(::robotiq_2f_gripper_msgs::action::MoveTwoFingerGripper_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_MoveTwoFingerGripper_GetResult_Response_result(msg_);
  }

private:
  ::robotiq_2f_gripper_msgs::action::MoveTwoFingerGripper_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::robotiq_2f_gripper_msgs::action::MoveTwoFingerGripper_GetResult_Response>()
{
  return robotiq_2f_gripper_msgs::action::builder::Init_MoveTwoFingerGripper_GetResult_Response_status();
}

}  // namespace robotiq_2f_gripper_msgs


namespace robotiq_2f_gripper_msgs
{

namespace action
{

namespace builder
{

class Init_MoveTwoFingerGripper_GetResult_Event_response
{
public:
  explicit Init_MoveTwoFingerGripper_GetResult_Event_response(::robotiq_2f_gripper_msgs::action::MoveTwoFingerGripper_GetResult_Event & msg)
  : msg_(msg)
  {}
  ::robotiq_2f_gripper_msgs::action::MoveTwoFingerGripper_GetResult_Event response(::robotiq_2f_gripper_msgs::action::MoveTwoFingerGripper_GetResult_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robotiq_2f_gripper_msgs::action::MoveTwoFingerGripper_GetResult_Event msg_;
};

class Init_MoveTwoFingerGripper_GetResult_Event_request
{
public:
  explicit Init_MoveTwoFingerGripper_GetResult_Event_request(::robotiq_2f_gripper_msgs::action::MoveTwoFingerGripper_GetResult_Event & msg)
  : msg_(msg)
  {}
  Init_MoveTwoFingerGripper_GetResult_Event_response request(::robotiq_2f_gripper_msgs::action::MoveTwoFingerGripper_GetResult_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_MoveTwoFingerGripper_GetResult_Event_response(msg_);
  }

private:
  ::robotiq_2f_gripper_msgs::action::MoveTwoFingerGripper_GetResult_Event msg_;
};

class Init_MoveTwoFingerGripper_GetResult_Event_info
{
public:
  Init_MoveTwoFingerGripper_GetResult_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MoveTwoFingerGripper_GetResult_Event_request info(::robotiq_2f_gripper_msgs::action::MoveTwoFingerGripper_GetResult_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_MoveTwoFingerGripper_GetResult_Event_request(msg_);
  }

private:
  ::robotiq_2f_gripper_msgs::action::MoveTwoFingerGripper_GetResult_Event msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::robotiq_2f_gripper_msgs::action::MoveTwoFingerGripper_GetResult_Event>()
{
  return robotiq_2f_gripper_msgs::action::builder::Init_MoveTwoFingerGripper_GetResult_Event_info();
}

}  // namespace robotiq_2f_gripper_msgs


namespace robotiq_2f_gripper_msgs
{

namespace action
{

namespace builder
{

class Init_MoveTwoFingerGripper_FeedbackMessage_feedback
{
public:
  explicit Init_MoveTwoFingerGripper_FeedbackMessage_feedback(::robotiq_2f_gripper_msgs::action::MoveTwoFingerGripper_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::robotiq_2f_gripper_msgs::action::MoveTwoFingerGripper_FeedbackMessage feedback(::robotiq_2f_gripper_msgs::action::MoveTwoFingerGripper_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robotiq_2f_gripper_msgs::action::MoveTwoFingerGripper_FeedbackMessage msg_;
};

class Init_MoveTwoFingerGripper_FeedbackMessage_goal_id
{
public:
  Init_MoveTwoFingerGripper_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MoveTwoFingerGripper_FeedbackMessage_feedback goal_id(::robotiq_2f_gripper_msgs::action::MoveTwoFingerGripper_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_MoveTwoFingerGripper_FeedbackMessage_feedback(msg_);
  }

private:
  ::robotiq_2f_gripper_msgs::action::MoveTwoFingerGripper_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::robotiq_2f_gripper_msgs::action::MoveTwoFingerGripper_FeedbackMessage>()
{
  return robotiq_2f_gripper_msgs::action::builder::Init_MoveTwoFingerGripper_FeedbackMessage_goal_id();
}

}  // namespace robotiq_2f_gripper_msgs

#endif  // ROBOTIQ_2F_GRIPPER_MSGS__ACTION__DETAIL__MOVE_TWO_FINGER_GRIPPER__BUILDER_HPP_
