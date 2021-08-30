// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_vacuum_gripper/franka_vacuum_gripper.h>

#include <cmath>
#include <functional>
#include <thread>
#include <chrono>

#include <ros/node_handle.h>

#include <franka/exception.h>
#include <franka/vacuum_gripper_state.h>
#include <franka_vacuum_gripper/VacuumAction.h>
#include <franka_vacuum_gripper/StopAction.h>
#include <franka_vacuum_gripper/DropOffAction.h>


namespace franka_vacuum_gripper {

bool updateGripperState(const franka::VacuumGripper& gripper, franka::VacuumGripperState* state) {
  try {
    *state = gripper.readOnce();
  } catch (const franka::Exception& ex) {
    ROS_ERROR_STREAM("GripperServer: Exception reading gripper state: " << ex.what());
    return false;
  }
  return true;
}

bool stop(const franka::VacuumGripper& gripper, const StopGoalConstPtr& /*goal*/) {
  return gripper.stop();
}

bool vacuum(const franka::VacuumGripper& gripper, const VacuumGoalConstPtr& goal) {
	// Transform timeout in int8 to std::chrono::milliseconds
	std::chrono::milliseconds timeout_ms(goal->timeout);
  return gripper.vacuum(goal->vacuum, timeout_ms);
}

bool dropOff(const franka::VacuumGripper& gripper, const DropOffGoalConstPtr& goal) {
	// Transform timeout in int8 to std::chrono::milliseconds
	std::chrono::milliseconds timeout_ms(goal->timeout);
  return gripper.dropOff(timeout_ms);
}
}  // namespace franka_gripper
