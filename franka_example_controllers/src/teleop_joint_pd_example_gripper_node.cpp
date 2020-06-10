// Copyright (c) 2020 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka/gripper.h>
#include <franka_gripper/GraspAction.h>
#include <franka_gripper/HomingAction.h>
#include <franka_gripper/MoveAction.h>
#include <franka_gripper/StopAction.h>

#include <actionlib/client/simple_action_client.h>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <sensor_msgs/JointState.h>
#include <signal.h>

#include <functional>
#include <mutex>

using franka_gripper::GraspAction;
using franka_gripper::HomingAction;
using franka_gripper::MoveAction;
using franka_gripper::StopAction;
using GraspClient = actionlib::SimpleActionClient<GraspAction>;
using HomingClient = actionlib::SimpleActionClient<HomingAction>;
using MoveClient = actionlib::SimpleActionClient<MoveAction>;
using StopClient = actionlib::SimpleActionClient<StopAction>;

class TeleopGripperClient {
 public:
  TeleopGripperClient()
      : master_homing_client_("master/homing", true),
        slave_homing_client_("slave/homing", true),
        grasp_client_("slave/grasp", true),
        move_client_("slave/move", true),
        stop_client_("slave/stop", true){};

  bool init(ros::NodeHandle& pnh) {
    grasping_ = false;
    gripper_homed_ = false;
    if (!pnh.getParam("gripper_homed", gripper_homed_)) {
      ROS_INFO_STREAM(
          "teleop_joint_pd_example_gripper_node: Could not read parameter gripper_homed. "
          "Defaulting to "
          << std::boolalpha << gripper_homed_);
    }

    if (!pnh.getParam("gripper/grasp_force", grasp_force_)) {
      ROS_ERROR(
          "teleop_joint_pd_example_gripper_node: Could not read parameter gripper/grasp_force. "
          "Aborting init!");
      return false;
    }

    if (!pnh.getParam("gripper/grasp_speed", grasp_speed_)) {
      ROS_ERROR(
          "teleop_joint_pd_example_gripper_node: Could not read parameter gripper/grasp_speed. "
          "Aborting init!");
      return false;
    }

    if (!pnh.getParam("gripper/move_speed", move_speed_)) {
      ROS_ERROR(
          "teleop_joint_pd_example_gripper_node: Could not read parameter gripper/move_speed. "
          "Aborting init!");
      return false;
    }

    bool homing_success(false);
    if (!gripper_homed_) {
      ROS_INFO("teleop_joint_pd_example_gripper_node: Homing Gripper.");
      homing_success = homingGripper_();
    }

    if (gripper_homed_ || homing_success) {
      // Start action servers and subscriber for gripper
      ros::Duration timeout(2.0);
      if (grasp_client_.waitForServer(timeout) && move_client_.waitForServer(timeout) &&
          stop_client_.waitForServer(timeout)) {
        master_sub_ = pnh.subscribe("master/joint_states", 1,
                                    &TeleopGripperClient::subscriberCallback_, this);
        return true;
      } else {
        ROS_ERROR(
            "teleop_joint_pd_example_gripper_node: Action Server could not be started. Shutting "
            "down node.");
        return false;
      }
    } else {
      return false;
    }
  };

 private:
  double max_width_{0.07};  // Default value. It will be reset when gripper is homed [m]
  bool grasping_;
  bool gripper_homed_;

  double grasp_force_;
  double grasp_speed_;
  double grasp_epsilon_inner_{0.001};  // [m]
  double grasp_epsilon_outer_scaling_{0.9};
  double move_speed_;

  double start_grasping_{0.5};  // Threshold position of master gripper where to start grasping.
  double start_opening_{0.6};   // Threshold position of master gripper where to open.

  HomingClient slave_homing_client_;
  HomingClient master_homing_client_;
  GraspClient grasp_client_;
  MoveClient move_client_;
  StopClient stop_client_;

  std::mutex subscriber_mutex_;
  ros::Subscriber master_sub_;

  bool homingGripper_() {
    if (slave_homing_client_.waitForServer(ros::Duration(2.0)) &&
        master_homing_client_.waitForServer(ros::Duration(2.0))) {
      master_homing_client_.sendGoal(franka_gripper::HomingGoal());
      slave_homing_client_.sendGoal(franka_gripper::HomingGoal());

      if (master_homing_client_.waitForResult(ros::Duration(10.0)) &&
          slave_homing_client_.waitForResult(ros::Duration(10.0))) {
        return true;
      }
    }
    ROS_ERROR("teleop_joint_pd_example_gripper_node: HomingAction has timed out.");
    return false;
  }

  void subscriberCallback_(const sensor_msgs::JointState& msg) {
    std::lock_guard<std::mutex> _(subscriber_mutex_);
    if (!gripper_homed_) {
      // If gripper had to be homed, reset max_width_.
      max_width_ = 2 * msg.position[0];
      gripper_homed_ = true;
    }
    double gripper_width = 2 * msg.position[0];
    if (gripper_width < start_grasping_ * max_width_ && !grasping_) {
      // Grasp object
      franka_gripper::GraspGoal grasp_goal;
      grasp_goal.force = grasp_force_;
      grasp_goal.speed = grasp_speed_;
      grasp_goal.epsilon.inner = grasp_epsilon_inner_;
      grasp_goal.epsilon.outer = grasp_epsilon_outer_scaling_ * max_width_;

      grasp_client_.sendGoal(grasp_goal);
      if (grasp_client_.waitForResult(ros::Duration(5.0))) {
        grasping_ = true;
      } else {
        ROS_INFO("teleop_joint_pd_example_gripper_node: GraspAction was not successful.");
        stop_client_.sendGoal(franka_gripper::StopGoal());
      }
    } else if (gripper_width > start_opening_ * max_width_ && grasping_) {
      // Open gripper
      franka_gripper::MoveGoal move_goal;
      move_goal.speed = move_speed_;
      move_goal.width = max_width_;
      move_client_.sendGoal(move_goal);
      if (move_client_.waitForResult(ros::Duration(5.0))) {
        grasping_ = false;
      } else {
        ROS_ERROR("teleop_joint_pd_example_gripper_node: MoveAction was not successful.");
        stop_client_.sendGoal(franka_gripper::StopGoal());
      }
    }
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "teleop_joint_pd_example_gripper_node");
  ros::NodeHandle pnh("~");
  TeleopGripperClient teleop_gripper_client;
  if (teleop_gripper_client.init(pnh)) {
    ros::spin();
  }
  return 0;
}
