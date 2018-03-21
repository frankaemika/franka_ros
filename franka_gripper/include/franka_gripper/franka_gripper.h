// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
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
 *
 * @return True if update was successful, false otherwise.
 */
bool updateGripperState(const franka::Gripper& gripper, franka::GripperState* state);

/**
 * Wraps the execution of a gripper command action to catch exceptions and
 * report results
 *
 * @note
 * For compatibility with current MoveIt! behavior, the given goal's command position is
 * multiplied by a factor of 2 before being commanded to the gripper!
 *
 * @param[in] gripper A pointer to a franka gripper
 * @param[in] default_speed The default speed for a gripper action
 * @param[in] grasp_epsilon The epsilon window of the grasp.
 * @param[in] action_server A pointer to a gripper action server
 * @param[in] goal A gripper action goal
 */
void gripperCommandExecuteCallback(
    const franka::Gripper& gripper,
    const GraspEpsilon& grasp_epsilon,
    double default_speed,
    actionlib::SimpleActionServer<control_msgs::GripperCommandAction>* action_server,
    const control_msgs::GripperCommandGoalConstPtr& goal);

/**
 * Calls the libfranka move service of the gripper
 *
 * @param[in] gripper A gripper instance to execute the command
 * @param[in] goal A move goal with target width and velocity
 *
 * @return True if command was successful, false otherwise.
 */
bool move(const franka::Gripper& gripper, const MoveGoalConstPtr& goal);

/**
 * Calls the libfranka homing service of the gripper
 *
 * @param[in] gripper A gripper instance to execute the command
 *
 * @return True if command was successful, false otherwise.
 */
bool homing(const franka::Gripper& gripper, const HomingGoalConstPtr& /*goal*/);

/**
 * Calls the libfranka stop service of the gripper to stop applying force
 *
 * @param[in] gripper A gripper instance to execute the command
 *
 * @return True if command was successful, false otherwise.
 */
bool stop(const franka::Gripper& gripper, const StopGoalConstPtr& /*goal*/);

/**
 * Calls the libfranka grasp service of the gripper
 *
 * An object is considered grasped if the distance \f$d\f$ between the gripper fingers satisfies
 * \f$(\text{width} - \text{epsilon_inner}) < d < (\text{width} + \text{epsilon_outer})\f$.
 *
 * @param[in] gripper A gripper instance to execute the command
 * @param[in] goal A grasp goal with target width, epsilon_inner, epsilon_outer, velocity and effort
 * @return True if an object has been grasped, false otherwise.
 */
bool grasp(const franka::Gripper& gripper, const GraspGoalConstPtr& goal);

}  // namespace franka_gripper
