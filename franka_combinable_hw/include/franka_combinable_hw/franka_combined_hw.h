// Copyright (c) 2019 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <combined_robot_hw/combined_robot_hw.h>
#include <franka_combinable_hw/franka_combinable_hw.h>
#include <franka_control/ErrorRecoveryAction.h>

#include <actionlib/server/simple_action_server.h>
#include <ros/node_handle.h>
#include <ros/time.h>

#include <memory>

namespace franka_combinable_hw {

class FrankaCombinedHW : public combined_robot_hw::CombinedRobotHW {
 public:
  /**
   * Creates an instance of CombinedFrankaHW
   *
   */
  FrankaCombinedHW();

  ~FrankaCombinedHW() {}

  /**
   * The init function is called to initialize the CombinedFrankaHW from a
   * non-realtime thread.
   *
   * @param[in] root_nh A NodeHandle in the root of the caller namespace.
   * @param[in] robot_hw_nh A NodeHandle in the namespace from which the RobotHW
   * should read its configuration.
   * @return True if initialization was successful
   */
  virtual bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) override;

  /**
   * Reads data from the robot HW
   *
   * @param[in] time The current time
   * @param[in] period The time passed since the last call to \ref read
   */
  virtual void read(const ros::Time& time, const ros::Duration& period) override;

  /**
   * Returns whether the controller needs to be reset.
   */
  bool controllerNeedsReset();

 protected:
  std::unique_ptr<actionlib::SimpleActionServer<franka_control::ErrorRecoveryAction>>
      dual_recovery_action_server_;

 private:
  void handleError();
  bool hasError();
  void triggerError();
  bool is_recovering_{false};
};

}  // namespace franka_combinable_hw
