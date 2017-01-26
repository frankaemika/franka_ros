#include <sensor_msgs/JointState.h>
#include "ros/ros.h"
// include lib_franka ...

int main(int argc, char **argv) {
  ros::init(argc, argv, "simple_viz_node");
  ros::NodeHandle nh("~");
  ros::NodeHandle nhp;
  ros::Rate rate(1000);
  ros::Publisher joint_pub =
      nhp.advertise<sensor_msgs::JointState>("joint_state", 1);
  std::vector<std::string> joint_names;
  sensor_msgs::JointState states;

  // parse yaml with joint names into joint_names..
  XmlRpc::XmlRpcValue params;
  nh.getParam("joint_names", params);
  joint_names.resize(params.size());
  for (int i = 0; i < params.size(); ++i) {
    joint_names[i] = static_cast<std::string>(params[i]);
    ROS_INFO("joint %d: %s", i, joint_names[i].c_str());
  }

  states.effort.resize(joint_names.size());
  states.name.resize(joint_names.size());
  states.position.resize(joint_names.size());
  states.velocity.resize(joint_names.size());

  std::vector<double> dummy;
  dummy.resize(joint_names.size());
  for (int i = 0; i < dummy.size(); ++i) {
    dummy[i] = 0.0f;
  }

  while (ros::ok()) {
    // read sensors from franka using libfranka

    // update joint_states to msgs
    states.header.stamp = ros::Time::now();
    for (int i = 0; i < joint_names.size(); ++i) {
      states.name.at(i) = joint_names.at(i);
      states.position.at(i) = dummy.at(i);
      states.velocity.at(i) = dummy.at(i);
      states.effort.at(i) = dummy.at(i);
    }

    joint_pub.publish(states);
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
