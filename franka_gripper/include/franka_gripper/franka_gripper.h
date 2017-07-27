#pragma once

#include <cmath>
#include <string>

#include <actionlib/server/simple_action_server.h>
#include <control_msgs/GripperCommandAction.h>
#include <ros/node_handle.h>

#include <franka/exception.h>
#include <franka/gripper.h>
#include <franka/gripper_state.h>
#include <franka_gripper/GraspAction.h>
#include <franka_gripper/HomingAction.h>
#include <franka_gripper/MoveAction.h>
#include <franka_gripper/StopAction.h>

namespace franka_gripper {

class GripperServer {
 public:
  GripperServer() = delete;

  /**
  * Constructs an instance of GripperServer
  *
  * @param[in] gripper An instance of a franka gripper
  * @param[in] node_handle A node handle to register the action servers
  * @param[in] width_tolerance A tolerance for reporting a successful action
  * @param[in] default_speed Speed for action execution
  * @param[in] newton_to_m_ampere_factor Mapping factor from Newton command to
  * milli Ampere
  */
  GripperServer(franka::Gripper& gripper,
                ros::NodeHandle& node_handle,
                double width_tolerance = 0.005,
                double default_speed = 0.1,
                double newton_to_m_ampere_factor = 14.9);
  /**
  * Reads the current gripper state from the hardware
  *
  * @param[in] state A pointer to store the new gripper state
  */
  bool getGripperState(franka::GripperState* state);

 private:
  /**
  * Executes a gripper command action
  *
  * @param[in] goal A gripper action goal
  * @param[in] state A gripper state
  */
  bool gripperCommandHandler(
      const control_msgs::GripperCommandGoalConstPtr& goal,
      franka::GripperState* state);

  /**
  * Wraps the execution of a gripper command action to catch exceptions and
  * report results
  *
  * @param[in] goal A gripper action goal
  */
  void gripperCommandexecuteCallback(
      const control_msgs::GripperCommandGoalConstPtr& goal);

  franka::Gripper* gripper_;
  const double default_speed_;
  const double newton_to_m_ampere_factor_;
  const double width_tolerance_;
  actionlib::SimpleActionServer<control_msgs::GripperCommandAction>
      gripper_command_action_server_;
};

/**
* Calls the libfranka move service of the gripper
*
* @param[in] gripper A gripper instance to execute the command
* @param[in] width_tolerance Tolerance width for reporting success
* @param[in] goal A move goal with target width and velocity
*/
bool move(franka::Gripper* gripper,
          double width_tolerance,
          const MoveGoalConstPtr& goal);

/**
* Calls the libfranka homing service of the gripper
*
* @param[in] gripper A gripper instance to execute the command
*/
bool homing(franka::Gripper* gripper, const HomingGoalConstPtr& /*goal*/);

/**
* Calls the libfranka stop service of the gripper to stop applying force
*
* @param[in] gripper A gripper instance to execute the command
*/
bool stop(franka::Gripper* gripper, const StopGoalConstPtr& /*goal*/);

/**
* Calls the libfranka grasp service of the gripper
*
* @param[in] gripper A gripper instance to execute the command
* @param[in] goal A grasp goal with target width, velocity and effort
*/
bool grasp(franka::Gripper* gripper, const GraspGoalConstPtr& goal);

}  // namespace franka_gripper
