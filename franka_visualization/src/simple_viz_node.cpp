#include <franka/robot.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "simple_viz_node");
  ros::NodeHandle nh("~");
  ros::NodeHandle nhp;
  ros::Rate rate(1000);
  ros::Publisher joint_pub =
      nhp.advertise<sensor_msgs::JointState>("joint_states", 1);
  std::vector<std::string> joint_names;

  // parse yaml with joint names and robot-IP
  XmlRpc::XmlRpcValue params;
  nh.getParam("joint_names", params);
  joint_names.resize(params.size());
  for (int i = 0; i < params.size(); ++i) {
    joint_names[i] = static_cast<std::string>(params[i]);
    ROS_INFO("parsed jointname[%d]= %s", i, joint_names[i].c_str());
  }
  std::string robot_ip;
  nh.getParam("robot_ip", robot_ip);
  ROS_INFO("parsed franka robot IP: %s", robot_ip.c_str());

  sensor_msgs::JointState states;
  states.effort.resize(joint_names.size());
  states.name.resize(joint_names.size());
  states.position.resize(joint_names.size());
  states.velocity.resize(joint_names.size());

  try {
    ROS_INFO("connecting to robot... ");
    franka::Robot robot(robot_ip);
    uint64_t secnr = 1;

    while (ros::ok() && robot.waitForRobotState()) {
      // read sensors from franka using libfranka
      const franka::RobotState& robot_state = robot.robotState();

      // update joint_states to msgs
      states.header.stamp = ros::Time::now();
      states.header.seq = secnr;
      for (int i = 0; i < joint_names.size(); ++i) {
        states.name.at(i) = joint_names.at(i);
        states.position.at(i) = robot_state.q[i];
        states.velocity.at(i) = robot_state.dq[i];
        states.effort.at(i) = robot_state.tau_J[i];
      }
      joint_pub.publish(states);
      ros::spinOnce();
      rate.sleep();
      secnr++;
    }
  } catch (franka::NetworkException const& e) {
    ROS_ERROR_STREAM("" << e.what());
    return -1;
  }

  return 0;
}
