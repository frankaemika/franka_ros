// Copyright (c) 2019 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_hw/franka_combined_hw.h>

#include <algorithm>
#include <memory>

#include <actionlib/server/simple_action_server.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <std_srvs/Trigger.h>

#include <franka_hw/franka_combinable_hw.h>
#include <franka_hw/franka_hw.h>
#include <franka_msgs/ErrorRecoveryAction.h>

namespace franka_hw {

FrankaCombinedHW::FrankaCombinedHW() = default;

bool FrankaCombinedHW::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) {
  bool success = CombinedRobotHW::init(root_nh, robot_hw_nh);
  // Error recovery server for all FrankaHWs
  combined_recovery_action_server_ =
      std::make_unique<actionlib::SimpleActionServer<franka_msgs::ErrorRecoveryAction>>(
          robot_hw_nh, "error_recovery",
          [&](const franka_msgs::ErrorRecoveryGoalConstPtr&) {
            try {
              is_recovering_ = true;
              for (const auto& robot_hw : robot_hw_list_) {
                auto* franka_combinable_hw_ptr =
                    dynamic_cast<franka_hw::FrankaCombinableHW*>(robot_hw.get());
                if (franka_combinable_hw_ptr != nullptr && franka_combinable_hw_ptr->connected()) {
                  franka_combinable_hw_ptr->resetError();
                } else {
                  ROS_ERROR("FrankaCombinedHW: failed to reset error. Is the robot connected?");
                  is_recovering_ = false;
                  combined_recovery_action_server_->setAborted(
                      franka_msgs::ErrorRecoveryResult(),
                      "dynamic_cast from RobotHW to FrankaCombinableHW failed");
                  return;
                }
              }
              is_recovering_ = false;
              combined_recovery_action_server_->setSucceeded();
            } catch (const franka::Exception& ex) {
              is_recovering_ = false;
              combined_recovery_action_server_->setAborted(franka_msgs::ErrorRecoveryResult(),
                                                           ex.what());
            }
          },
          false);
  combined_recovery_action_server_->start();

  connect_server_ =
      robot_hw_nh.advertiseService<std_srvs::Trigger::Request, std_srvs::Trigger::Response>(
          "connect",
          [this](std_srvs::Trigger::Request& request,
                 std_srvs::Trigger::Response& response) -> bool {
            try {
              connect();
              ROS_INFO("FrankaCombinedHW: successfully connected robots.");
              response.success = 1u;
              response.message = "";
            } catch (const std::exception& e) {
              ROS_INFO("Combined: exception %s", e.what());
              response.success = 0u;
              response.message =
                  "FrankaCombinedHW: Failed to connect robot: " + std::string(e.what());
            }
            return true;
          });

  disconnect_server_ =
      robot_hw_nh.advertiseService<std_srvs::Trigger::Request, std_srvs::Trigger::Response>(
          "disconnect",
          [this](std_srvs::Trigger::Request& request,
                 std_srvs::Trigger::Response& response) -> bool {
            bool success = disconnect();
            response.success = success ? 1u : 0u;
            response.message = success
                                   ? "FrankaCombinedHW: Successfully disconnected robots."
                                   : "FrankaCombinedHW: Failed to disconnect robots. All active "
                                     "controllers must be stopped before you can disconnect.";
            if (success) {
              ROS_INFO("%s", response.message.c_str());
            } else {
              ROS_ERROR("%s", response.message.c_str());
            }
            return true;
          });

  return success;
}

void FrankaCombinedHW::read(const ros::Time& time, const ros::Duration& period) {
  // Call the read method of the single RobotHW objects.
  CombinedRobotHW::read(time, period);
  handleError();
}

bool FrankaCombinedHW::controllerNeedsReset() {
  // Check if any of the RobotHW object needs a controller reset
  bool controller_reset = false;
  for (const auto& robot_hw : robot_hw_list_) {
    auto* franka_combinable_hw_ptr = dynamic_cast<franka_hw::FrankaCombinableHW*>(robot_hw.get());
    if (franka_combinable_hw_ptr != nullptr) {
      controller_reset = controller_reset || franka_combinable_hw_ptr->controllerNeedsReset();
    } else {
      ROS_ERROR("FrankaCombinedHW: dynamic_cast from RobotHW to FrankaCombinableHW failed.");
      return false;
    }
  }
  return controller_reset;
}

void FrankaCombinedHW::handleError() {
  // Trigger error state of all other RobotHW objects when one of them has a error.
  if (hasError() && !is_recovering_) {
    triggerError();
  }
}

bool FrankaCombinedHW::hasError() {
  bool has_error = false;
  for (const auto& robot_hw : robot_hw_list_) {
    auto* franka_combinable_hw_ptr = dynamic_cast<franka_hw::FrankaCombinableHW*>(robot_hw.get());
    if (franka_combinable_hw_ptr != nullptr) {
      has_error = has_error || franka_combinable_hw_ptr->hasError();
    } else {
      ROS_ERROR("FrankaCombinedHW: dynamic_cast from RobotHW to FrankaCombinableHW failed.");
      return false;
    }
  }
  return has_error;
}

void FrankaCombinedHW::triggerError() {
  // Trigger error state of all RobotHW objects.
  for (const auto& robot_hw : robot_hw_list_) {
    auto* franka_combinable_hw_ptr = dynamic_cast<franka_hw::FrankaCombinableHW*>(robot_hw.get());
    if (franka_combinable_hw_ptr != nullptr) {
      franka_combinable_hw_ptr->triggerError();
    } else {
      ROS_ERROR("FrankaCombinedHW: dynamic_cast from RobotHW to FrankaCombinableHW failed.");
    }
  }
}

void FrankaCombinedHW::connect() {
  for (const auto& robot_hw : robot_hw_list_) {
    auto* franka_combinable_hw_ptr = dynamic_cast<franka_hw::FrankaCombinableHW*>(robot_hw.get());
    if (franka_combinable_hw_ptr != nullptr && !franka_combinable_hw_ptr->connected()) {
      franka_combinable_hw_ptr->connect();
    }
  }
}

bool FrankaCombinedHW::disconnect() {
  // Ensure all robots are disconnectable (not running a controller)
  for (const auto& robot_hw : robot_hw_list_) {
    auto* franka_combinable_hw_ptr = dynamic_cast<franka_hw::FrankaCombinableHW*>(robot_hw.get());
    if (franka_combinable_hw_ptr != nullptr && franka_combinable_hw_ptr->controllerActive()) {
      return false;
    }
  }

  // Only if all robots are in fact disconnectable, disconnecting them.
  // Fail and abort if any robot cannot be disconnected.
  for (const auto& robot_hw : robot_hw_list_) {
    auto* franka_combinable_hw_ptr = dynamic_cast<franka_hw::FrankaCombinableHW*>(robot_hw.get());
    if (franka_combinable_hw_ptr != nullptr && !franka_combinable_hw_ptr->disconnect()) {
      return false;
    }
  }

  return true;
}

}  // namespace franka_hw
