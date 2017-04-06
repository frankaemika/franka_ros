#include <string>
#include <vector>

#include <franka_hw/franka_hw.h>

#include <franka/robot.h>
#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "franka_hw");
  ros::NodeHandle nh("~");

  // parse robot_hw yaml with joint names and robot-IP:
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
  franka_hw::FrankaHW franka_ros(joint_names, robot_ip,
                                 franka_states_publish_rate, nh);

  return static_cast<int>(!franka_ros.update([cycle_start = ros::Time::now()](const franka::RobotState&) mutable {
    ROS_INFO_THROTTLE(1, "cycle: %f s",
                      (ros::Time::now() - cycle_start).toSec());
    cycle_start = ros::Time::now();
    return ros::ok();
  }));
}
