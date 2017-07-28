#include <franka_gripper/franka_gripper.h>

#include <cmath>
#include <functional>
#include <thread>

#include <ros/node_handle.h>

#include <franka/exception.h>
#include <franka/gripper_state.h>
#include <franka_gripper/GraspAction.h>
#include <franka_gripper/HomingAction.h>
#include <franka_gripper/MoveAction.h>
#include <franka_gripper/StopAction.h>

namespace franka_gripper {

bool getGripperState(franka::GripperState* state, franka::Gripper* gripper) {
  try {
    *state = gripper->readOnce();
  } catch (const franka::Exception& ex) {
    ROS_ERROR_STREAM(
        "GripperServer: Exception reading gripper state: " << ex.what());
    return false;
  }
  return true;
}

void gripperCommandExecuteCallback(
    franka::Gripper* gripper,
    const double default_speed,
    const double newton_to_m_ampere_factor,
    actionlib::SimpleActionServer<control_msgs::GripperCommandAction>*
        action_server,
    const control_msgs::GripperCommandGoalConstPtr& goal) {
  std::function<bool()> gripper_command_handler = [=]() {
    franka::GripperState state = gripper->readOnce();
    if (goal->command.position > state.max_width ||
        goal->command.position < 0.0) {
      ROS_ERROR_STREAM(
          "GripperServer: Commanding out of range width! max_width = "
          << state.max_width << " command = " << goal->command.position);
      return false;
    }
    if (goal->command.position >= state.width) {
      gripper->move(goal->command.position, default_speed);
      return true;
    } else {
      return gripper->grasp(
          goal->command.position, default_speed,
          goal->command.max_effort * newton_to_m_ampere_factor);
    }
  };

  try {
    if (gripper_command_handler()) {
      franka::GripperState state;
      if (getGripperState(&state, gripper)) {
        control_msgs::GripperCommandResult result;
        result.effort = 0.0;
        result.position = state.width;
        result.reached_goal = static_cast<decltype(result.reached_goal)>(true);
        result.stalled = static_cast<decltype(result.stalled)>(false);
        action_server->setSucceeded(result);
        return;
      }
    }
  } catch (const franka::Exception& ex) {
    ROS_ERROR_STREAM("" << ex.what());
  }
  action_server->setAborted();
}

bool move(franka::Gripper* gripper, const MoveGoalConstPtr& goal) {
  gripper->move(goal->width, goal->speed);
  return true;
}

bool homing(franka::Gripper* gripper, const HomingGoalConstPtr& /*goal*/) {
  gripper->homing();
  return true;
}

bool stop(franka::Gripper* gripper, const StopGoalConstPtr& /*goal*/) {
  gripper->stop();
  return true;
}

bool grasp(franka::Gripper* gripper, const GraspGoalConstPtr& goal) {
  return gripper->grasp(goal->width, goal->speed, goal->max_current);
}

}  // namespace franka_gripper
