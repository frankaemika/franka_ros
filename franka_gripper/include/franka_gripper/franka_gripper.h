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

/**
  * Reads a gripper state if possible
  *
  * @param[in] state A gripper state to update
  * @param[in] gripper A pointer to a franka gripper
  */
bool updateGripperState(const franka::Gripper& gripper,
                        franka::GripperState* state);

/**
  * Wraps the execution of a gripper command action to catch exceptions and
  * report results
  *
  * @param[in] gripper A pointer to a franka gripper
  * @param[in] default_speed The default speed for a gripper action
  * @param[in] newton_to_m_ampere_factor The mapping factor from Newton to milli
  * Ampere
  * @param[in] action_server A pointer to a gripper action server
  * @param[in] goal A gripper action goal
  */
void gripperCommandExecuteCallback(
    const franka::Gripper& gripper,
    double default_speed,
    double newton_to_m_ampere_factor,
    actionlib::SimpleActionServer<control_msgs::GripperCommandAction>*
        action_server,
    const control_msgs::GripperCommandGoalConstPtr& goal);

/**
 * Calls the libfranka move service of the gripper
 *
 * @param[in] gripper A gripper instance to execute the command
 * @param[in] goal A move goal with target width and velocity
 */
bool move(const franka::Gripper& gripper, const MoveGoalConstPtr& goal);

/**
 * Calls the libfranka homing service of the gripper
 *
 * @param[in] gripper A gripper instance to execute the command
 */
bool homing(const franka::Gripper& gripper, const HomingGoalConstPtr& /*goal*/);

/**
 * Calls the libfranka stop service of the gripper to stop applying force
 *
 * @param[in] gripper A gripper instance to execute the command
 */
bool stop(const franka::Gripper& gripper, const StopGoalConstPtr& /*goal*/);

/**
 * Calls the libfranka grasp service of the gripper
 *
 * @param[in] gripper A gripper instance to execute the command
 * @param[in] goal A grasp goal with target width, velocity and effort
 */
bool grasp(const franka::Gripper& gripper, const GraspGoalConstPtr& goal);

}  // namespace franka_gripper
