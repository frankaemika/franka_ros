#pragma once

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <franka_gripper/GraspAction.h>
#include <franka_gripper/HomingAction.h>
#include <franka_gripper/MoveAction.h>
#include <gtest/gtest.h>
#include <ros/ros.h>

class GripperSimTestSetup : public ::testing::Test {
 protected:
  std::unique_ptr<actionlib::SimpleActionClient<::franka_gripper::HomingAction>> homing_client;
  std::unique_ptr<actionlib::SimpleActionClient<::franka_gripper::MoveAction>> move_client;
  std::unique_ptr<actionlib::SimpleActionClient<::franka_gripper::GraspAction>> grasp_client;
  ros::NodeHandle n;
  double finger_1_pos = 0.;
  double finger_2_pos = 0.;
  double finger_1_force = 0.;
  double finger_2_force = 0.;
  void SetUp() override;
  void updateFingerState();
  void resetStone();
  void setRealTimeUpdateRate(double rate);
};
