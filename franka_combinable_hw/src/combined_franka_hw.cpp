// Copyright (c) 2018 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_combinable_hw/franka_combinable_hw.h>

#include <franka_combinable_hw/combined_franka_hw.h>
#include <franka_hw/franka_hw.h>
#include <franka_control/ErrorRecoveryAction.h>

#include <actionlib/server/simple_action_server.h>
#include <ros/time.h>
#include <ros/node_handle.h>

#include <algorithm>
#include <memory>

namespace franka_combinable_hw {

CombinedFrankaHW::CombinedFrankaHW() = default;

bool CombinedFrankaHW::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) {
  bool success = CombinedRobotHW::init(root_nh, robot_hw_nh);
  // Error recovery server for all FrankaHWs
  dual_recovery_action_server_ =
      std::make_unique<actionlib::SimpleActionServer<franka_control::ErrorRecoveryAction> >(
          robot_hw_nh, "error_recovery",
          [&](const franka_control::ErrorRecoveryGoalConstPtr&) {
            try {
              is_recovering_ = true;
              for (const auto& robot_hw : robot_hw_list_) {
                auto* franka_combinable_hw_ptr = dynamic_cast<franka_combinable_hw::FrankaCombinableHW*>(robot_hw.get());
                if (franka_combinable_hw_ptr != nullptr) {
                  franka_combinable_hw_ptr->resetError();
                } else {
                  ROS_ERROR("CombinedFrankaHW: dynamic_cast from RobotHW to FrankaCombinableHW failed.");
                  is_recovering_ = false;
                  dual_recovery_action_server_->setAborted(
                      franka_control::ErrorRecoveryResult(),
                      "dynamic_cast from RobotHW to FrankaCombinableHW failed");
                  return;
                }
              }
              is_recovering_ = false;
              dual_recovery_action_server_->setSucceeded();
            } catch (const franka::Exception& ex) {
              is_recovering_ = false;
              dual_recovery_action_server_->setAborted(franka_control::ErrorRecoveryResult(), ex.what());
            }
          },
          false);
  dual_recovery_action_server_->start();
  return success;
}

void CombinedFrankaHW::read(const ros::Time& time, const ros::Duration& period) {
  // Call the read method of the single RobotHW objects.
  CombinedRobotHW::read(time, period);
  handleError();
}

bool CombinedFrankaHW::controllerNeedsReset() {
  // Check if any of the RobotHW object needs a controller reset
  bool controller_reset = false;
  for (const auto& robot_hw : robot_hw_list_) {
    auto* franka_combinable_hw_ptr = dynamic_cast<franka_combinable_hw::FrankaCombinableHW*>(robot_hw.get());
    if (franka_combinable_hw_ptr != nullptr) {
      controller_reset = controller_reset || franka_combinable_hw_ptr->controllerNeedsReset();
    } else {
      ROS_ERROR("CombinedFrankaHW: dynamic_cast from RobotHW to FrankaCombinableHW failed.");
      return false;
    }
  }
  return controller_reset;
}

void CombinedFrankaHW::handleError() {
  // Trigger error state of all other RobotHW objects when one of them has a error.
  if (hasError() && !is_recovering_) {
    triggerError();
  }
}

bool CombinedFrankaHW::hasError() {
  // TODO(qu_zh): save FrankaCombinableHW vector instead of RobotHW, if dynamic_cast is too expensive
  bool has_error = false;
  for (const auto& robot_hw : robot_hw_list_) {
    auto* franka_combinable_hw_ptr = dynamic_cast<franka_combinable_hw::FrankaCombinableHW*>(robot_hw.get());
    if (franka_combinable_hw_ptr != nullptr) {
      has_error = has_error || franka_combinable_hw_ptr->hasError();
    } else {
      ROS_ERROR("CombinedFrankaHW: dynamic_cast from RobotHW to FrankaCombinableHW failed.");
      return false;
    }
  }
  return has_error;
}

void CombinedFrankaHW::triggerError() {
  // Trigger error state of all RobotHW objects.
  for (const auto& robot_hw : robot_hw_list_) {
    auto* franka_combinable_hw_ptr = dynamic_cast<franka_combinable_hw::FrankaCombinableHW*>(robot_hw.get());
    if (franka_combinable_hw_ptr != nullptr) {
      franka_combinable_hw_ptr->triggerError();
    } else {
      ROS_ERROR("CombinedFrankaHW: dynamic_cast from RobotHW to FrankaCombinableHW failed.");
    }
  }
}

}  // namespace franka_combinable_hw
