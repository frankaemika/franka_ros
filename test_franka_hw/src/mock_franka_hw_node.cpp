#include <array>
#include <cassert>
#include <string>
#include <vector>

#include <controller_manager/controller_manager.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <ros/ros.h>
#include <ros/spinner.h>
#include <urdf/model.h>
#include <xmlrpcpp/XmlRpcValue.h>

#include <franka/robot.h>
#include <franka_hw/franka_hw.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "mock_franka_hw");
  ros::NodeHandle node_handle("~");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  XmlRpc::XmlRpcValue params;
  node_handle.getParam("joint_names", params);
  std::vector<std::string> joint_names(params.size());
  for (int i = 0; i < params.size(); ++i) {
    joint_names[i] = static_cast<std::string>(params[i]);
  }
  double franka_states_publish_rate(30.0);
  node_handle.getParam("franka_states_publish_rate",
                       franka_states_publish_rate);

  franka::Robot* empty_robot_pointer = nullptr;
  std::string arm_id;
  node_handle.getParam("arm_id", arm_id);

  franka_hw::FrankaHW franka_ros(joint_names, empty_robot_pointer,
                                 franka_states_publish_rate, arm_id,
                                 node_handle);
  ROS_INFO("Initialized franka hw mock");

  std::vector<joint_limits_interface::JointLimits> joint_limits;
  joint_limits.resize(7);

  urdf::Model urdf_model;
  if (!urdf_model.initParamWithNodeHandle("robot_description", node_handle)) {
    ROS_ERROR(
        "Could not initialize urdf_model parsing robot_description in "
        "MockHWnode");
  } else {
    ROS_INFO("Succesfully initialized urdf model in MockHWnode");
  }

  for (size_t i = 0; i < joint_names.size(); ++i) {
    boost::shared_ptr<const urdf::Joint> urdf_joint =
        urdf_model.getJoint(joint_names[i]);
    ROS_INFO_STREAM("Got Joint with limits: effort="
                    << urdf_joint->limits->effort
                    << " upper=" << urdf_joint->limits->upper
                    << " lower=" << urdf_joint->limits->lower << " velocity"
                    << urdf_joint->limits->velocity);
    if (!joint_limits_interface::getJointLimits(urdf_joint, joint_limits[i])) {
      ROS_ERROR_STREAM("Could not parse joint limits of joint "
                       << joint_names[i]);
      return -1;
    }
  }

  std::array<double, 7> position_command;
  std::array<double, 7> velocity_command;
  std::array<double, 7> effort_command;
  ros::Duration period(0.001);
  ros::Rate rate(1000);
  controller_manager::ControllerManager ctrl_manager(&franka_ros);

  ROS_INFO("Starting control loop!");
  ros::Time now(ros::Time::now());
  ros::Time last(ros::Time::now());
  while (ros::ok()) {
    now = ros::Time::now();
    period = now - last;
    last = now;
    ctrl_manager.update(now, period);
    franka_ros.enforceLimits(period);
    position_command = franka_ros.getJointPositionCommand();
    velocity_command = franka_ros.getJointVelocityCommand();
    effort_command = franka_ros.getJointEffortCommand();

    ROS_INFO_STREAM("hw_node corrected commands pos vel eff = \n"
                    << position_command[0] << " " << position_command[1] << " "
                    << position_command[2] << " " << position_command[3] << " "
                    << position_command[4] << " " << position_command[5] << " "
                    << position_command[6] << " ;\n " << velocity_command[0]
                    << " " << velocity_command[1] << " " << velocity_command[2]
                    << " " << velocity_command[3] << " " << velocity_command[4]
                    << " " << velocity_command[5] << " " << velocity_command[6]
                    << " ;\n " << effort_command[0] << " " << effort_command[1]
                    << " " << effort_command[2] << " " << effort_command[3]
                    << " " << effort_command[4] << " " << effort_command[5]
                    << " " << effort_command[6]);
    for (size_t i = 0; i < 7; ++i) {
      assert((position_command[i] <= joint_limits[i].max_position &&
              position_command[i] >= joint_limits[i].min_position &&
              "Position limits exceeded"));
      assert((velocity_command[i] <= joint_limits[i].max_velocity &&
              velocity_command[i] >= -joint_limits[i].max_velocity &&
              "Velocity limits exceeded"));
      assert((effort_command[i] <= joint_limits[i].max_effort &&
              effort_command[i] >= -joint_limits[i].max_effort &&
              "Effort limits exceeded"));
    }
    rate.sleep();
  }
  spinner.stop();
  return 0;
}
