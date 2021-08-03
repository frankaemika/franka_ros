// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <algorithm>
#include <atomic>
#include <chrono>
#include <thread>

#include <actionlib/server/simple_action_server.h>
#include <controller_manager/controller_manager.h>
#include <franka/exception.h>
#include <franka/robot.h>
#include <franka_hw/franka_hw.h>
#include <franka_hw/services.h>
#include <franka_msgs/ErrorRecoveryAction.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>

using franka_hw::ServiceContainer;
using namespace std::chrono_literals;

int main(int argc, char** argv) {
  ros::init(argc, argv, "franka_control_node");

  ros::NodeHandle public_node_handle;
  ros::NodeHandle node_handle("~");

  franka_hw::FrankaHW franka_control;
  if (!franka_control.init(public_node_handle, node_handle)) {
    ROS_ERROR("franka_control_node: Failed to initialize FrankaHW class. Shutting down!");
    return 1;
  }

  auto services = std::make_unique<ServiceContainer>();
  std::unique_ptr<actionlib::SimpleActionServer<franka_msgs::ErrorRecoveryAction>>
      recovery_action_server;

  std::atomic_bool has_error(false);

  auto connect = [&]() {
    franka_control.connect();
    std::lock_guard<std::mutex> lock(franka_control.robotMutex());
    auto& robot = franka_control.robot();

    services = std::make_unique<ServiceContainer>();
    franka_hw::setupServices(robot, franka_control.robotMutex(), node_handle, *services);

    recovery_action_server =
        std::make_unique<actionlib::SimpleActionServer<franka_msgs::ErrorRecoveryAction>>(
            node_handle, "error_recovery",
            [&](const franka_msgs::ErrorRecoveryGoalConstPtr&) {
              try {
                std::lock_guard<std::mutex> lock(franka_control.robotMutex());
                robot.automaticErrorRecovery();
                has_error = false;
                recovery_action_server->setSucceeded();
                ROS_INFO("Recovered from error");
              } catch (const franka::Exception& ex) {
                recovery_action_server->setAborted(franka_msgs::ErrorRecoveryResult(), ex.what());
              }
            },
            false);

    recovery_action_server->start();

    // Initialize robot state before loading any controller
    franka_control.update(robot.readOnce());
  };

  auto disconnect_handler = [&](std_srvs::Trigger::Request& request,
                                std_srvs::Trigger::Response& response) -> bool {
    if (franka_control.controllerActive()) {
      response.success = 0u;
      response.message = "Controller is active. Cannot disconnect while a controller is running.";
      return true;
    }
    services.reset();
    recovery_action_server.reset();
    auto result = franka_control.disconnect();
    response.success = result ? 1u : 0u;
    response.message = result ? "" : "Failed to disconnect robot.";
    return true;
  };

  auto connect_handler = [&](std_srvs::Trigger::Request& request,
                             std_srvs::Trigger::Response& response) -> bool {
    if (franka_control.connected()) {
      response.success = 0u;
      response.message = "Already connected to robot. Cannot connect twice.";
      return true;
    }

    connect();

    response.success = 1u;
    response.message = "";
    return true;
  };

  connect();

  ros::ServiceServer connect_server =
      node_handle.advertiseService<std_srvs::Trigger::Request, std_srvs::Trigger::Response>(
          "connect", connect_handler);
  ros::ServiceServer disconnect_server =
      node_handle.advertiseService<std_srvs::Trigger::Request, std_srvs::Trigger::Response>(
          "disconnect", disconnect_handler);

  controller_manager::ControllerManager control_manager(&franka_control, public_node_handle);

  // Start background threads for message handling
  ros::AsyncSpinner spinner(4);
  spinner.start();

  while (ros::ok()) {
    ros::Time last_time = ros::Time::now();

    // Wait until controller has been activated or error has been recovered
    while (!franka_control.controllerActive() || has_error) {
      if (franka_control.connected()) {
        try {
          std::lock_guard<std::mutex> lock(franka_control.robotMutex());
          franka_control.update(franka_control.robot().readOnce());
          ros::Time now = ros::Time::now();
          control_manager.update(now, now - last_time);
          franka_control.checkJointLimits();
          last_time = now;
        } catch (const std::logic_error& e) {
        }
      } else {
        std::this_thread::sleep_for(1ms);
      }

      if (!ros::ok()) {
        return 0;
      }
    }

    if (franka_control.connected()) {
      try {
        // Run control loop. Will exit if the controller is switched.
        franka_control.control([&](const ros::Time& now, const ros::Duration& period) {
          if (period.toSec() == 0.0) {
            // Reset controllers before starting a motion
            control_manager.update(now, period, true);
            franka_control.checkJointLimits();
            franka_control.reset();
          } else {
            control_manager.update(now, period);
            franka_control.checkJointLimits();
            franka_control.enforceLimits(period);
          }
          return ros::ok();
        });
      } catch (const franka::ControlException& e) {
        ROS_ERROR("%s", e.what());
        has_error = true;
      }
    }
    ROS_INFO_THROTTLE(1, "franka_control, main loop");
  }

  return 0;
}
