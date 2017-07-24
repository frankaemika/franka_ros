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

  /**
  * Constructs an instance of GripperServer
  *
  * @param[in] robot_ip The IP address of the robot
  * @param[in] node_handle A nodehandle to parse parameters and publish
  */
  GripperServer(const std::string& robot_ip, ros::NodeHandle& node_handle);

  /**
  * Reads the current gripper state from the hardware
  *
  * @param[in] state A pointer to store the new gripper state
  */
  bool getGripperState(franka::GripperState* state);

 private:
  /**
  * Calls the libfranka move service of the gripper
  *
  * @param[in] request A move command with target width and velocity
  */
  void move(const Move::Request& request);

  /**
  * Calls the libfranka homing service of the gripper
  */
  void homing();

  /**
  * Calls the libfranka stop service of the gripper to stop applying force
  */
  void stop();

  /**
  * Calls the libfranka grasp service of the gripper
  *
  * @param[in] request A grasp command with target width,velocity and max effort
  */
  void grasp(const Grasp::Request& request);

  /**
  * A callback function for a control_msgs/GripperCommand action
  *
  * @param[in] goal A gripper action goal
  */
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
