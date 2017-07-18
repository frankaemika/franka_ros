#pragma once

#include <string>

#include <ros/service_server.h>
#include <ros/node_handle.h>

#include <franka_gripper/Grasp.h>
#include <franka_gripper/Homing.h>
#include <franka_gripper/Stop.h>
#include <franka_gripper/Move.h>
#include <franka/gripper.h>

namespace franka_gripper {

class GripperServer {
public:
  GripperServer() = delete;
  GripperServer(const std::string& robot_ip, ros::NodeHandle &node_handle);

private:
  void move(const Move::Request& request);
  void homing();
  void stop();
  void grasp(const Grasp::Request& request);

  franka::Gripper gripper_;
  ros::ServiceServer grasp_server_;
  ros::ServiceServer homing_server_;
  ros::ServiceServer move_server_;
  ros::ServiceServer stop_server_;
};

}  // namespace franka_gripper
