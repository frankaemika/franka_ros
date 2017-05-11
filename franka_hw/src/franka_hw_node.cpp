
#include <string>
#include <vector>

#include <ros/ros.h>
#include <xmlrpcpp/XmlRpc.h>

#include <franka/robot.h>
#include <franka_hw/franka_hw.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "franka_hw");
  ros::NodeHandle node_handle("~");

  XmlRpc::XmlRpcValue params;
  node_handle.getParam("joint_names", params);
  std::vector<std::string> joint_names(params.size());
  for (int i = 0; i < params.size(); ++i) {
    joint_names[i] = static_cast<std::string>(params[i]);
  }
  std::string robot_ip;
  node_handle.getParam("robot_ip", robot_ip);
  std::string arm_id;
  node_handle.getParam("arm_id", arm_id);
  double franka_states_publish_rate = 30.0;
  node_handle.getParam("franka_states_publish_rate",
                       franka_states_publish_rate);
  franka::Robot robot(robot_ip);
  franka_hw::FrankaHW franka_ros(
      joint_names, &robot, franka_states_publish_rate, arm_id, node_handle);

  return static_cast<int>(!franka_ros.update([cycle_start = ros::Time::now()](
      const franka::RobotState&) mutable {
    ROS_INFO_THROTTLE(1, "cycle: %f s",
                      (ros::Time::now() - cycle_start).toSec());
    cycle_start = ros::Time::now();
    return ros::ok();
  }));
}
