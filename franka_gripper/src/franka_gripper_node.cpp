#include <string>
#include <thread>
#include <vector>

#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/rate.h>
#include <ros/spinner.h>
#include <sensor_msgs/JointState.h>
#include <xmlrpcpp/XmlRpc.h>

#include <franka/gripper_state.h>
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
  double publish_rate(30.0);
  if (!node_handle.getParam("publish_rate", publish_rate)) {
    ROS_INFO_STREAM(
        "franka_gripper_node: Could not find parameter publish_rate. "
        "Defaulting to "
        << publish_rate);
  }

  XmlRpc::XmlRpcValue params;
  if (!node_handle.getParam("joint_names", params)) {
    ROS_ERROR("franka_gripper_node: Could not parse joint_names!");
    return -1;
  }
  if (params.size() != 2) {
    ROS_ERROR("franka_gripper_node: Got wrong number of joint_names!");
    return -1;
  }
  std::vector<std::string> joint_names(params.size());
  for (int i = 0; i < params.size(); ++i) {
    joint_names[i] = static_cast<std::string>(params[i]);
  }

  franka::GripperState gripper_state;
  std::thread read_thread([&gripper_state, &gripper_server]() {
    ros::Rate read_rate(10);
    franka::GripperState new_gripper_state;
    while (ros::ok()) {
      if (gripper_server.getGripperState(&new_gripper_state)) {
        gripper_state = new_gripper_state;
      }
      read_rate.sleep();
    }
  });

  ros::Publisher gripper_state_publisher =
      node_handle.advertise<sensor_msgs::JointState>("joint_states", 1);
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::Rate rate(publish_rate);
  while (ros::ok()) {
    sensor_msgs::JointState joint_states;
    joint_states.header.stamp = ros::Time::now();
    joint_states.name.push_back(joint_names[0]);
    joint_states.name.push_back(joint_names[1]);
    joint_states.position.push_back(gripper_state.width * 0.5);
    joint_states.position.push_back(gripper_state.width * 0.5);
    joint_states.velocity.push_back(0.0);
    joint_states.velocity.push_back(0.0);
    joint_states.effort.push_back(0.0);
    joint_states.effort.push_back(0.0);
    gripper_state_publisher.publish(joint_states);
    rate.sleep();
  }
  read_thread.join();
  return 0;
}
