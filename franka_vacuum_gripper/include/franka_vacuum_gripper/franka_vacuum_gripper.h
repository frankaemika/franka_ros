// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <cmath>
#include <string>

#include <actionlib/server/simple_action_server.h>
#include <ros/node_handle.h>

#include <franka/exception.h>
#include <franka/vacuum_gripper.h>
#include <franka/vacuum_gripper_state.h>

#include <franka_vacuum_gripper/StopAction.h>
#include <franka_vacuum_gripper/VacuumAction.h>
#include <franka_vacuum_gripper/DropOffAction.h>

namespace franka_vacuum_gripper {

/**
 * Reads a vacuum gripper state if possible
 *
 * @param[in] state A gripper state to update
 * @param[in] gripper A pointer to a franka gripper
 *
 * @return True if update was successful, false otherwise.
 */
bool updateGripperState(const franka::VacuumGripper& gripper, franka::VacuumGripperState* state);

bool stop(const franka::VacuumGripper& gripper, const StopGoalConstPtr& /*goal*/);

/**
 * Calls the libfranka vacuum service of the vacuum gripper
 *
 * An object is considered grasped if the distance \f$d\f$ between the gripper fingers satisfies
 * \f$(\text{width} - \text{epsilon_inner}) < d < (\text{width} + \text{epsilon_outer})\f$.
 *
 * @param[in] gripper A gripper instance to execute the command
 * @param[in] goal A grasp goal with target vacuum and timeout
 * @return True if an object has been vacuumed, false otherwise.
 */
bool vacuum(const franka::VacuumGripper& gripper, const VacuumGoalConstPtr& goal);

/**
 * Calls the libfranka drop off service of the vacuum gripper
 *
 * @param[in] gripper A gripper instance to execute the command
 * @param[in] goal A dropoff goal with timeout
 * @return True if an object has been dropped, false otherwise.
 */
bool dropOff(const franka::VacuumGripper& gripper, const DropOffGoalConstPtr& goal);

}  // namespace franka_vacuum_gripper
