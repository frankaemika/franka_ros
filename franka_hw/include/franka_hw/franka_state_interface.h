// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <string>

#include <franka/robot_state.h>
#include <hardware_interface/internal/hardware_resource_manager.h>

namespace franka_hw {

/**
 * Handle to read the complete state of a robot.
 */
class FrankaStateHandle {
 public:
  FrankaStateHandle() = delete;

  /**
   * Creates an instance of a FrankaStateHandle.
   *
   * @param[in] name The name of the state handle.
   * @param[in] robot_state A reference to the robot state wrapped by this handle.
   */
  FrankaStateHandle(const std::string& name, franka::RobotState& robot_state)
      : name_(name), robot_state_(&robot_state) {}

  /**
   * Gets the name of the state handle.
   *
   * @return Name of the state handle.
   */
  const std::string& getName() const noexcept { return name_; }

  /**
   * Gets the current robot state.
   *
   * @return Current robot state.
   */
  const franka::RobotState& getRobotState() const noexcept { return *robot_state_; }

 private:
  std::string name_;
  const franka::RobotState* robot_state_;
};

/**
 * Hardware interface to read the complete robot state.
 *
 * @see franka::RobotState for a description of the values included in the robot state.
 */
class FrankaStateInterface : public hardware_interface::HardwareResourceManager<FrankaStateHandle> {
};

}  // namespace franka_hw
