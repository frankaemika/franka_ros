#pragma once

#include <string>

#include <actionlib/server/simple_action_server.h>
#include <control_msgs/GripperCommandAction.h>
#include <ros/node_handle.h>
#include <ros/service_server.h>

#include <franka/gripper.h>
#include <franka/gripper_state.h>
#include <franka_gripper/Grasp.h>
#include <franka_gripper/Homing.h>
#include <franka_gripper/Move.h>
#include <franka_gripper/Stop.h>

namespace franka_gripper {

class GripperServer {
 public:
  static constexpr double kCommandVelocity{0.1};
  static constexpr double kNewtonToMilliAmpereFactor{
      1.0};  // TODO CJ: Get precise value
  static constexpr double kWidthTolerance{0.005};

  GripperServer() = delete;
  GripperServer(const std::string& robot_ip, ros::NodeHandle& node_handle);
  bool getGripperState(franka::GripperState* state);

 private:
  void move(const Move::Request& request);
  void homing();
  void stop();
  void grasp(const Grasp::Request& request);
  void executeGripperCommand(
      const control_msgs::GripperCommandGoalConstPtr& goal);

  franka::Gripper gripper_;
  ros::ServiceServer grasp_server_;
  ros::ServiceServer homing_server_;
  ros::ServiceServer move_server_;
  ros::ServiceServer stop_server_;
  actionlib::SimpleActionServer<control_msgs::GripperCommandAction>
      action_server_;
};

}  // namespace franka_gripper
