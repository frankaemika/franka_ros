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

class ServiceContainer {
 public:
  template <typename T, typename... TArgs>
  ServiceContainer& advertiseService(TArgs&&... args) {
    ros::ServiceServer server = franka_control::advertiseService<T>(std::forward<TArgs>(args)...);
    services_.push_back(server);
    return *this;
  }

 private:
  std::vector<ros::ServiceServer> services_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "franka_control_node");
  ros::NodeHandle public_node_handle;
  ros::NodeHandle node_handle("~");

  std::vector<std::string> joint_names_vector;
  if (!node_handle.getParam("joint_names", joint_names_vector) || joint_names_vector.size() != 7) {
    ROS_ERROR("Invalid or no joint_names parameters provided");
    return 1;
  }

  std::array<std::string, 7> joint_names;
  std::copy(joint_names_vector.cbegin(), joint_names_vector.cend(), joint_names.begin());

  bool rate_limiting;
  if (!node_handle.getParamCached("rate_limiting", rate_limiting)) {
    ROS_ERROR("Invalid or no rate_limiting parameter provided");
    return 1;
  }

  double cutoff_frequency;
  if (!node_handle.getParamCached("cutoff_frequency", cutoff_frequency)) {
    ROS_ERROR("Invalid or no cutoff_frequency parameter provided");
    return 1;
  }

  std::string internal_controller;
  if (!node_handle.getParam("internal_controller", internal_controller)) {
    ROS_ERROR("No internal_controller parameter provided");
    return 1;
  }

  urdf::Model urdf_model;
  if (!urdf_model.initParamWithNodeHandle("robot_description", public_node_handle)) {
    ROS_ERROR("Could not initialize URDF model from robot_description");
    return 1;
  }

  std::string robot_ip;
  if (!node_handle.getParam("robot_ip", robot_ip)) {
    ROS_ERROR("Invalid or no robot_ip parameter provided");
    return 1;
  }

  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("Invalid or no arm_id parameter provided");
    return 1;
  }
  franka::Robot robot(robot_ip);

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

  franka::Model model = robot.loadModel();
  auto get_rate_limiting = [&]() {
    node_handle.getParamCached("rate_limiting", rate_limiting);
    return rate_limiting;
  };
  auto get_internal_controller = [&]() {
    node_handle.getParamCached("internal_controller", internal_controller);

    franka::ControllerMode controller_mode;
    if (internal_controller == "joint_impedance") {
      controller_mode = franka::ControllerMode::kJointImpedance;
    } else if (internal_controller == "cartesian_impedance") {
      controller_mode = franka::ControllerMode::kCartesianImpedance;
    } else {
      ROS_WARN("Invalid internal_controller parameter provided, falling back to joint impedance");
      controller_mode = franka::ControllerMode::kJointImpedance;
    }

    return controller_mode;
  };
  auto get_cutoff_frequency = [&]() {
    node_handle.getParamCached("cutoff_frequency", cutoff_frequency);
    return cutoff_frequency;
  };
  franka_hw::FrankaHW franka_control(joint_names, arm_id, urdf_model, model, get_rate_limiting,
                                     get_cutoff_frequency, get_internal_controller);

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
      last_time = now;

      if (!ros::ok()) {
        return 0;
      }
    }

    try {
      // Run control loop. Will exit if the controller is switched.
      franka_control.control(robot, [&](const ros::Time& now, const ros::Duration& period) {
        if (period.toSec() == 0.0) {
          // Reset controllers before starting a motion
          control_manager.update(now, period, true);
          franka_control.reset();
        } else {
          control_manager.update(now, period);
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
