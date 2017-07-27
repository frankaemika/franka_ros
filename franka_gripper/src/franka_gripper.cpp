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

using actionlib::SimpleActionServer;

GripperServer::GripperServer(franka::Gripper& gripper,
                             ros::NodeHandle& node_handle,
                             double width_tolerance,
                             double default_speed,
                             double newton_to_m_ampere_factor)
    : gripper_(&gripper),
      gripper_command_action_server_(
          node_handle,
          "gripper_action",
          std::bind(&GripperServer::gripperCommandexecuteCallback,
                    this,
                    std::placeholders::_1),
          false),
      width_tolerance_(width_tolerance),
      default_speed_(default_speed),
      newton_to_m_ampere_factor_(newton_to_m_ampere_factor) {
  gripper_command_action_server_.start();
}

bool GripperServer::getGripperState(franka::GripperState* state) {
  try {
    *state = gripper_->readOnce();
  } catch (const franka::Exception& ex) {
    ROS_ERROR_STREAM(
        "GripperServer: Exception reading gripper state: " << ex.what());
    return false;
  }
  return true;
}

bool GripperServer::gripperCommandHandler(
    const control_msgs::GripperCommandGoalConstPtr& goal,
    franka::GripperState* state) {
  *state = gripper_->readOnce();
  if (goal->command.position > state->max_width ||
      goal->command.position < 0.0) {
    ROS_ERROR_STREAM(
        "GripperServer: Commanding out of range width! max_width = "
        << state->max_width << " command = " << goal->command.position);
    return false;
  }
  if (goal->command.position >= state->width) {
    gripper_->move(goal->command.position, default_speed_);
    *state = gripper_->readOnce();
    if (std::fabs(state->width - goal->command.position) < width_tolerance_) {
      return true;
    }
    ROS_WARN("GripperServer: Move failed");
  } else {
    gripper_->grasp(goal->command.position, default_speed_,
                    goal->command.max_effort * newton_to_m_ampere_factor_);
    *state = gripper_->readOnce();
    if (state->is_grasped) {
      return true;
    }
    gripper_->stop();
    ROS_WARN("GripperServer: Grasp failed");
  }
  return false;
}

void GripperServer::gripperCommandexecuteCallback(
    const control_msgs::GripperCommandGoalConstPtr& goal) {
  franka::GripperState state;
  try {
    if (gripperCommandHandler(goal, &state)) {
      control_msgs::GripperCommandResult result;
      result.effort = 0.0;
      result.position = state.width;
      result.reached_goal = static_cast<decltype(result.reached_goal)>(true);
      result.stalled = static_cast<decltype(result.stalled)>(false);
      gripper_command_action_server_.setSucceeded(result);
      return;
    }
  } catch (const franka::Exception& ex) {
    ROS_ERROR_STREAM("" << ex.what());
  }
  gripper_command_action_server_.setAborted();
}

bool move(franka::Gripper* gripper,
          double width_tolerance,
          const MoveGoalConstPtr& goal) {
  gripper->move(goal->width, goal->speed);
  return (std::fabs(gripper->readOnce().width - goal->width) < width_tolerance);
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
  gripper->grasp(goal->width, goal->speed, goal->max_current);
  return gripper->readOnce().is_grasped;
}

}  // namespace franka_gripper
