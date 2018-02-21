// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka/exception.h>
#include <franka/robot.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "robot_joint_state_publisher");
  ros::NodeHandle node_handle("~");

  std::vector<std::string> joint_names;
  node_handle.getParam("joint_names", joint_names);

  std::string robot_ip;
  node_handle.getParam("robot_ip", robot_ip);

  double publish_rate;
  node_handle.getParam("publish_rate", publish_rate);

  ros::Rate rate(publish_rate);

  sensor_msgs::JointState states;
  states.effort.resize(joint_names.size());
  states.name.resize(joint_names.size());
  states.position.resize(joint_names.size());
  states.velocity.resize(joint_names.size());

  for (size_t i = 0; i < joint_names.size(); i++) {
    states.name[i] = joint_names[i];
  }

  ros::Publisher publisher = node_handle.advertise<sensor_msgs::JointState>("joint_states", 1);

  try {
    franka::Robot robot(robot_ip);

    robot.read([&](const franka::RobotState& robot_state) {
      states.header.stamp = ros::Time::now();
      for (size_t i = 0; i < joint_names.size(); i++) {
        states.position[i] = robot_state.q[i];
        states.velocity[i] = robot_state.dq[i];
        states.effort[i] = robot_state.tau_J[i];
      }
      publisher.publish(states);
      ros::spinOnce();
      rate.sleep();
      return ros::ok();
    });

  } catch (const franka::Exception& e) {
    ROS_ERROR_STREAM("Exception: " << e.what());
    return -1;
  }

  return 0;
}
