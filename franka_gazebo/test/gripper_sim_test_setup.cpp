#include "gripper_sim_test_setup.h"

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

void GripperSimTestSetup::SetUp() {
  homing_client = std::make_unique<actionlib::SimpleActionClient<franka_gripper::HomingAction>>(
      "franka_gripper/homing", true);
  move_client = std::make_unique<actionlib::SimpleActionClient<franka_gripper::MoveAction>>(
      "franka_gripper/move", true);
  grasp_client = std::make_unique<actionlib::SimpleActionClient<franka_gripper::GraspAction>>(
      "franka_gripper/grasp", true);
  homing_client->waitForServer();
  move_client->waitForServer();
  grasp_client->waitForServer();
  UpdateFingerPositions();
}
void GripperSimTestSetup::UpdateFingerPositions() {
  auto msg = ros::topic::waitForMessage<sensor_msgs::JointState>("/franka_gripper/joint_states", n);
  finger_1_pos = msg->position.at(0);
  finger_2_pos = msg->position.at(1);
}
