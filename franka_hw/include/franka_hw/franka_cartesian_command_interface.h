// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <array>

#include <franka_hw/franka_state_interface.h>
#include <hardware_interface/internal/hardware_resource_manager.h>

namespace franka_hw {

/**
 * Handle to read and command a Cartesian pose.
 */
class FrankaCartesianPoseHandle : public FrankaStateHandle {
 public:
  FrankaCartesianPoseHandle() = delete;

  /**
   * Creates an instance of a FrankaCartesianPoseHandle.
   *
   * @param[in] franka_state_handle Robot state handle.
   * @param[in] command A reference to the Cartesian pose command wrapped by this handle.
   * @param[in] elbow A reference to the elbow wrapped by this handle.
   */
  FrankaCartesianPoseHandle(const FrankaStateHandle& franka_state_handle,
                            std::array<double, 16>& command,
                            std::array<double, 2>& elbow)
      : FrankaStateHandle(franka_state_handle), command_(&command), elbow_(&elbow) {}

  /**
   * Sets the given command.
   *
   * @param[in] command Command to set.
   */
  void setCommand(const std::array<double, 16>& command) noexcept {
    *command_ = command;
    *elbow_ = {};
  }

  /**
   * Sets the given command.
   *
   * @param[in] command Command to set.
   * @param[in] elbow Elbow to set.
   */
  void setCommand(const std::array<double, 16>& command,
                  const std::array<double, 2>& elbow) noexcept {
    *command_ = command;
    *elbow_ = elbow;
  }

  /**
   * Gets the current command.
   *
   * @return Current command.
   */
  const std::array<double, 16>& getCommand() const noexcept { return *command_; }

 private:
  std::array<double, 16>* command_;
  std::array<double, 2>* elbow_;
};

/**
 * Hardware interface to command Cartesian poses.
 */
class FrankaPoseCartesianInterface
    : public hardware_interface::HardwareResourceManager<FrankaCartesianPoseHandle,
                                                         hardware_interface::ClaimResources> {};

/**
 * Handle to read and command a Cartesian velocity.
 */
class FrankaCartesianVelocityHandle : public FrankaStateHandle {
 public:
  FrankaCartesianVelocityHandle() = delete;

  /**
   * @param[in] franka_state_handle Robot state handle.
   * @param[in] command A reference to the Cartesian velocity command wrapped by this handle.
   * @param[in] elbow A reference to the elbow wrapped by this handle.
   */
  FrankaCartesianVelocityHandle(const FrankaStateHandle& franka_state_handle,
                                std::array<double, 6>& command,
                                std::array<double, 2>& elbow)
      : FrankaStateHandle(franka_state_handle), command_(&command), elbow_(&elbow) {}

  /**
   * Sets the given command.
   *
   * @param[in] command Command to set.
   */
  void setCommand(std::array<double, 6>& command) noexcept {
    *command_ = command;
    *elbow_ = {};
  }

  /**
   * Sets the given command.
   *
   * @param[in] command Command to set.
   * @param[in] elbow Elbow to set.
   */
  void setCommand(const std::array<double, 6>& command,
                  const std::array<double, 2>& elbow) noexcept {
    *command_ = command;
    *elbow_ = elbow;
  }

  /**
   * Gets the current command.
   *
   * @return Current command.
   */
  const std::array<double, 6>& getCommand() const noexcept { return *command_; }

 private:
  std::array<double, 6>* command_;
  std::array<double, 2>* elbow_;
};

/**
 * Hardware interface to command Cartesian velocities.
 */
class FrankaVelocityCartesianInterface
    : public hardware_interface::HardwareResourceManager<FrankaCartesianVelocityHandle,
                                                         hardware_interface::ClaimResources> {};

}  // namespace franka_hw
