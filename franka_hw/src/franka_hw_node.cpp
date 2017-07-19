#include <unistd.h>
#include <string>
#include <vector>

#include <controller_manager/controller_manager.h>
#include <ros/ros.h>
#include <ros/spinner.h>
#include <xmlrpcpp/XmlRpc.h>

#include <franka/model.h>
#include <franka/robot.h>
#include <franka_hw/franka_hw.h>
#include <franka_hw/service_server.h>
#include <franka_hw/trigger_rate.h>

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
  franka::Robot robot(robot_ip);
  franka_hw::ServiceServer service_server(robot, node_handle);
  franka_hw::FrankaHW franka_ros(joint_names, &robot, arm_id, node_handle);
  controller_manager::ControllerManager control_manager(&franka_ros,
                                                        node_handle);
  ros::AsyncSpinner spinner(1);
  spinner.start();

  const ros::Duration kPeriod(0.001);
  franka_ros.run([&]() {
    control_manager.update(ros::Time::now(), kPeriod);
    franka_ros.enforceLimits(kPeriod);
    return ros::ok();
  });
  return 0;
}
