#include <franka_gripper/franka_gripper.h>

#include <thread>
#include <cmath>
#include <functional>

#include <ros/node_handle.h>

#include <franka_gripper/GraspAction.h>
#include <franka_gripper/MoveAction.h>
#include <franka_gripper/StopAction.h>
#include <franka_gripper/HomingAction.h>
#include <franka/exception.h>
#include <franka/gripper_state.h>

namespace franka_gripper {


GripperServer::GripperServer(const std::string& robot_ip, ros::NodeHandle& node_handle)
    : gripper_(robot_ip),
      gripper_command_action_server_(node_handle,
                     "gripper_action",
                     std::bind(&GripperServer::gripperCommandexecuteCallback, this, std::placeholders::_1),
                     false) {
  gripper_command_action_server_.start();
}

bool GripperServer::getGripperState(franka::GripperState* state) {
  try {
    *state = gripper_.readOnce();
  } catch (const franka::Exception& ex) {
    ROS_ERROR_STREAM(
        "GripperServer: Exception reading gripper state: " << ex.what());
    return false;
  }
  return true;
}

bool GripperServer::move(const MoveActionGoalConstPtr& goal) {
    gripper_.move(goal->goal.width, goal->goal.speed);
    if (std::fabs(gripper_.readOnce().width - goal->goal.width) < width_tolerance_) {
       return true;
    }
    return false;
}

bool GripperServer::homing(const HomingActionGoalConstPtr& /*goal*/) {
  gripper_.homing();
  return true;
}

bool GripperServer::stop(const StopActionGoalConstPtr& /*goal*/) {
  gripper_.stop();
  return true;
}

bool GripperServer::grasp(const GraspActionGoalConstPtr& goal) {
  gripper_.grasp(goal->goal.width, goal->goal.speed, goal->goal.max_current);
  return gripper_.readOnce().is_grasped;
}

bool GripperServer::gripperCommandHandler(const control_msgs::GripperCommandGoalConstPtr& goal,
                                          franka::GripperState* state) {
    *state = gripper_.readOnce();
    if (goal->command.position > state->max_width ||
        goal->command.position < 0.0) {
      ROS_ERROR_STREAM(
          "GripperServer: Commanding out of range width! max_width = "
          << state->max_width << " command = " << goal->command.position);
      return false;
    }
    if (goal->command.position >= state->width) {
      gripper_.move(goal->command.position, default_speed_);
      *state = gripper_.readOnce();
      if (std::pow((state->width - goal->command.position), 2) <
          width_tolerance_) {
        return true;
      }
      ROS_WARN("GripperServer: Move failed");
    } else {
      gripper_.grasp(goal->command.position, default_speed_,
                     goal->command.max_effort * newton_to_m_ampere_factor_);
      *state = gripper_.readOnce();
      if (state->is_grasped) {
        return true;
      }
      gripper_.stop();
      ROS_WARN("GripperServer: Grasp failed");
    }
   return false;
}

void GripperServer::gripperCommandexecuteCallback(const control_msgs::GripperCommandGoalConstPtr& goal) {
   franka::GripperState state;
   try {
       if (gripperCommandHandler(goal, &state)) {
           control_msgs::GripperCommandResult result;
           result.effort = 0.0;
           result.position = state.width;
           result.reached_goal = true;
           result.stalled = false;
           gripper_command_action_server_.setSucceeded(result);
           return;
       }
   } catch (const franka::Exception& ex) {
     ROS_ERROR_STREAM("" << ex.what());
   }
   gripper_command_action_server_.setAborted();
}

}  // namespace franka_gripper
