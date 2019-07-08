// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <algorithm>
#include <array>
#include <atomic>
#include <string>
#include <utility>

#include <actionlib/server/simple_action_server.h>
#include <controller_manager/controller_manager.h>
#include <franka/exception.h>
#include <franka/robot.h>
#include <franka_hw/franka_hw.h>
#include <ros/ros.h>

#include <franka_control/ErrorRecoveryAction.h>
#include <franka_control/services.h>

using franka_control::ServiceContainer;

int main(int argc, char** argv) {
  ros::init(argc, argv, "franka_control_node");

  // TODO hw node_handle + root_node_handle
  ros::NodeHandle public_node_handle;
  ros::NodeHandle node_handle("~");

  franka_hw::FrankaHW franka_control;
  if (!franka_control.init(public_node_handle, node_handle)) {
    ROS_ERROR("franka_control_node: Failed to initialize FrankaHW class. Shutting down!");
    return 1;
  }

  franka::Robot& robot = franka_control.robot();

  // Set default collision behavior
  robot.setCollisionBehavior(
      {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
      {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
      {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
      {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

  std::atomic_bool has_error(false);

  ServiceContainer services;
  services
      .advertiseService<franka_control::SetJointImpedance>(
          node_handle, "set_joint_impedance",
          [&robot](auto&& req, auto&& res) {
            return franka_control::setJointImpedance(robot, req, res);
          })
      .advertiseService<franka_control::SetCartesianImpedance>(
          node_handle, "set_cartesian_impedance",
          [&robot](auto&& req, auto&& res) {
            return franka_control::setCartesianImpedance(robot, req, res);
          })
      .advertiseService<franka_control::SetEEFrame>(
          node_handle, "set_EE_frame",
          [&robot](auto&& req, auto&& res) { return franka_control::setEEFrame(robot, req, res); })
      .advertiseService<franka_control::SetKFrame>(
          node_handle, "set_K_frame",
          [&robot](auto&& req, auto&& res) { return franka_control::setKFrame(robot, req, res); })
      .advertiseService<franka_control::SetForceTorqueCollisionBehavior>(
          node_handle, "set_force_torque_collision_behavior",
          [&robot](auto&& req, auto&& res) {
            return franka_control::setForceTorqueCollisionBehavior(robot, req, res);
          })
      .advertiseService<franka_control::SetFullCollisionBehavior>(
          node_handle, "set_full_collision_behavior",
          [&robot](auto&& req, auto&& res) {
            return franka_control::setFullCollisionBehavior(robot, req, res);
          })
      .advertiseService<franka_control::SetLoad>(
          node_handle, "set_load",
          [&robot](auto&& req, auto&& res) { return franka_control::setLoad(robot, req, res); });

  actionlib::SimpleActionServer<franka_control::ErrorRecoveryAction> recovery_action_server(
      node_handle, "error_recovery",
      [&](const franka_control::ErrorRecoveryGoalConstPtr&) {
        try {
          robot.automaticErrorRecovery();
          has_error = false;
          recovery_action_server.setSucceeded();
          ROS_INFO("Recovered from error");
        } catch (const franka::Exception& ex) {
          recovery_action_server.setAborted(franka_control::ErrorRecoveryResult(), ex.what());
        }
      },
      false);

  // Initialize robot state before loading any controller
  franka_control.update(robot.readOnce());

  controller_manager::ControllerManager control_manager(&franka_control, public_node_handle);

  recovery_action_server.start();

  // Start background threads for message handling
  ros::AsyncSpinner spinner(4);
  spinner.start();

  while (ros::ok()) {
    ros::Time last_time = ros::Time::now();

    // Wait until controller has been activated or error has been recovered
    while (!franka_control.controllerActive() || has_error) {
      franka_control.update(robot.readOnce());

      ros::Time now = ros::Time::now();
      control_manager.update(now, now - last_time);
      franka_control.checkJointLimits();
      last_time = now;

      if (!ros::ok()) {
        return 0;
      }
    }

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

  return 0;
}
