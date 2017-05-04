#include <string>
#include <vector>

#include <franka_hw/franka_hw.h>
#include <xmlrpcpp/XmlRpc.h>

#include <franka/robot.h>
#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "franka_hw");
  ros::NodeHandle nh("~");

  XmlRpc::XmlRpcValue params;
  nh.getParam("joint_names", params);
  std::vector<std::string> joint_names(params.size());
  for (int i = 0; i < params.size(); ++i) {
    joint_names[i] = static_cast<std::string>(params[i]);
  }
  std::string robot_ip;
  nh.getParam("robot_ip", robot_ip);
  double franka_states_publish_rate = 30.0;
  nh.getParam("franka_states_publish_rate", franka_states_publish_rate);
  franka::Robot robot(robot_ip);
  franka_hw::FrankaHW franka_ros(joint_names, &robot,
                                 franka_states_publish_rate, nh);
  ros::Duration period(0.0);
  while (ros::ok()) {
    ros::Time cycle_start(ros::Time::now());
    if (!franka_ros.update(period)) {
      ROS_ERROR("failed to update franka_hw. Shutting down hardware node!");
      return -1;
    }
    period = ros::Time::now() - cycle_start;
  }
  return 0;
}
