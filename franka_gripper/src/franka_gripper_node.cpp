#include <string>

#include <ros/init.h>
#include <ros/node_handle.h>

#include <franka_gripper/franka_gripper.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "franka_gripper_node");
  ros::NodeHandle node_handle("~");
  std::string robot_ip;
  if (!node_handle.getParam("robot_ip", robot_ip)) {
    ROS_ERROR("franka_gripper_node: Could not parse robot_ip parameter");
    return -1;
  }
  franka_gripper::GripperServer gripper_server(robot_ip, node_handle);
  ros::spin();

  return 0;
}
