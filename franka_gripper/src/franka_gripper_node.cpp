
#include <string>

#include <ros/node_handle.h>
#include <ros/spinner.h>

#include <franka/exception.h>
#include <franka_gripper/franka_gripper.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "franka_gripper_node");
  ros::NodeHandle node_handle("~");

  std::string robot_ip;
  if (!node_handle.getParam("robot_ip", robot_ip)) {
    ROS_ERROR("franka_gripper_node: Could not parse robot_ip parameter");
    return -1;
  }

  try {
    franka_gripper::GripperServer gripper_server(robot_ip, node_handle);
    ros::spin();
  } catch (franka::NetworkException& ex) {
    ROS_ERROR_STREAM(
        "franka_gripper_node: NetworkException creating gripper_server: "
        << ex.what());
    return -1;
  } catch (franka::IncompatibleVersionException& ex) {
    ROS_ERROR_STREAM(
        "franka_gripper_node: IncompatibleVersionException creating "
        "gripper_server: "
        << ex.what());
    return -1;
  } catch (franka::ProtocolException& ex) {
    ROS_ERROR_STREAM(
        "franka_gripper_node: ProtocolException creating gripper_server: "
        << ex.what());
    return -1;
  }

  return 0;
}
