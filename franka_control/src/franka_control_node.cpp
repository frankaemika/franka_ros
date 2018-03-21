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

  using std::placeholders::_1;
  using std::placeholders::_2;
  ServiceContainer services;
  services
      .advertiseService<franka_control::SetJointImpedance>(
          node_handle, "set_joint_impedance",
          std::bind(franka_control::setJointImpedance, std::ref(robot), _1, _2))
      .advertiseService<franka_control::SetCartesianImpedance>(
          node_handle, "set_cartesian_impedance",
          std::bind(franka_control::setCartesianImpedance, std::ref(robot), _1, _2))
      .advertiseService<franka_control::SetEEFrame>(
          node_handle, "set_EE_frame",
          std::bind(franka_control::setEEFrame, std::ref(robot), _1, _2))
      .advertiseService<franka_control::SetKFrame>(
          node_handle, "set_K_frame", std::bind(franka_control::setKFrame, std::ref(robot), _1, _2))
      .advertiseService<franka_control::SetForceTorqueCollisionBehavior>(
          node_handle, "set_force_torque_collision_behavior",
          std::bind(franka_control::setForceTorqueCollisionBehavior, std::ref(robot), _1, _2))
      .advertiseService<franka_control::SetFullCollisionBehavior>(
          node_handle, "set_full_collision_behavior",
          std::bind(franka_control::setFullCollisionBehavior, std::ref(robot), _1, _2))
      .advertiseService<franka_control::SetLoad>(
          node_handle, "set_load", std::bind(franka_control::setLoad, std::ref(robot), _1, _2));

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
  franka_hw::FrankaHW franka_control(joint_names, arm_id, public_node_handle, model);

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
