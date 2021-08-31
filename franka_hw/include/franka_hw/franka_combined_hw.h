// Copyright (c) 2019 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <combined_robot_hw/combined_robot_hw.h>
#include <franka_hw/franka_combinable_hw.h>
#include <franka_msgs/ErrorRecoveryAction.h>

#include <actionlib/server/simple_action_server.h>
#include <ros/node_handle.h>
#include <ros/time.h>

#include <memory>

namespace franka_hw {

class FrankaCombinedHW : public combined_robot_hw::CombinedRobotHW {
 public:
  /**
   * Creates an instance of CombinedFrankaHW.
   */
  FrankaCombinedHW();

  ~FrankaCombinedHW() override = default;  // NOLINT (clang-analyzer-optin.cplusplus.VirtualCall)

  /**
   * The init function is called to initialize the CombinedFrankaHW from a
   * non-realtime thread.
   *
   * @param[in] root_nh A NodeHandle in the root of the caller namespace.
   * @param[in] robot_hw_nh A NodeHandle in the namespace from which the RobotHW.
   * should read its configuration.
   * @return True if initialization was successful.
   */
  bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) override;

  /**
   * Reads data from the robot HW
   *
   * @param[in] time The current time.
   * @param[in] period The time passed since the last call to \ref read.
   */
  void read(const ros::Time& time, const ros::Duration& period) override;

  /**
   * Checks whether the controller needs to be reset.
   *
   * @return True if the controllers needs to be reset, false otherwise.
   */
  bool controllerNeedsReset();

  /**
   * Calls connect on all hardware classes that are of type `FrankaCombinableHW`.
   */
  void connect();

  /**
   * Tries to disconnect on all hardware classes that are of type `FrankaCombinableHW`.
   * @return true if successful, false otherwise.
   */
  bool disconnect();

  /**
   * Checks whether the robots are in error or reflex mode.
   * @return true if in error state, false otherwise.
   */
  bool hasError();

 protected:
  std::unique_ptr<actionlib::SimpleActionServer<franka_msgs::ErrorRecoveryAction>>
      combined_recovery_action_server_;
  ros::ServiceServer connect_server_;
  ros::ServiceServer disconnect_server_;

 private:
  void handleError();
  void triggerError();
  bool is_recovering_{false};
};

}  // namespace franka_hw
