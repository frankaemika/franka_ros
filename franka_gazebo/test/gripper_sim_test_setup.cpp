#include "gripper_sim_test_setup.h"
#include <gazebo_msgs/SetModelState.h>
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
  resetStone();
  UpdateFingerState();
}

void GripperSimTestSetup::UpdateFingerState() {
  auto msg = ros::topic::waitForMessage<sensor_msgs::JointState>("/franka_gripper/joint_states", n);
  finger_1_pos = msg->position.at(0);
  finger_2_pos = msg->position.at(1);
  finger_1_force = msg->effort.at(0);
  finger_2_force = msg->effort.at(1);
}

void GripperSimTestSetup::resetStone() {
  ros::ServiceClient service =
      n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
  gazebo_msgs::SetModelState srv;
  srv.request.model_state.model_name = "stone";
  srv.request.model_state.pose.position.x = 0.000286;
  srv.request.model_state.pose.position.y = -0.221972;
  srv.request.model_state.pose.position.z = 0.475121;
  srv.request.model_state.reference_frame = "world";
  service.call(srv);
  if (not srv.response.success) {
    ROS_ERROR("resetting stone failed");
  }
}
