#include <unistd.h>
#include <string>
#include <vector>

#include <controller_manager/controller_manager.h>
#include <ros/ros.h>
#include <ros/spinner.h>
#include <xmlrpcpp/XmlRpc.h>

#include <franka/robot.h>
#include <franka_hw/franka_hw.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "franka_hw");
  ros::NodeHandle node_handle("~");
  ros::AsyncSpinner spinner(1);
  spinner.start();

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
  controller_manager::ControllerManager control_manager(&franka_ros);

  ros::Duration period(0.0);
  ros::Time now(ros::Time::now());
  ros::Time last(ros::Time::now());

  std::function<void(void)> ros_callback = [&]() {
    now = ros::Time::now();
    period = now - last;
    last = now;
    control_manager.update(now, period);
    franka_ros.enforceLimits(period);
    franka_ros.publishExternalWrench();
    franka_ros.publishFrankaStates();
    franka_ros.publishJointStates();
    franka_ros.publishTransforms();
  };

  while (ros::ok()) {
    franka_ros.run(ros_callback);
    ros_callback();
    usleep(5000);
  }
  spinner.stop();
  return 0;
}
