#include <franka_example_controllers/joint_velocity_example_controller.h>

#include <cmath>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <xmlrpcpp/XmlRpcValue.h>

namespace franka_example_controllers {

JointVelocityExampleController::JointVelocityExampleController()
    : velocity_joint_interface_(nullptr) {}

bool JointVelocityExampleController::init(hardware_interface::RobotHW* robot_hardware,
                                          ros::NodeHandle& root_node_handle,
                                          ros::NodeHandle& /*controller_node_handle*/) {
  velocity_joint_interface_ = robot_hardware->get<hardware_interface::VelocityJointInterface>();
  if (velocity_joint_interface_ == nullptr) {
    ROS_ERROR(
        "JointVelocityExampleController: Error getting velocity joint interface from hardware!");
    return false;
  }
  XmlRpc::XmlRpcValue parameters;
  if (!root_node_handle.getParam("joint_names", parameters)) {
    ROS_ERROR("JointVelocityExampleController: Could not parse joint names");
  }
  if (parameters.size() != 7) {
    ROS_ERROR_STREAM("JointVelocityExampleController: Wrong number of joint names, got "
                     << int(parameters.size()) << " instead of 7 names!");
    return false;
  }
  velocity_joint_handles_.resize(7);
  joint_names_.resize(7);
  for (size_t i = 0; i < 7; ++i) {
    joint_names_[i] = static_cast<std::string>(parameters[i]);
    try {
      velocity_joint_handles_[i] = velocity_joint_interface_->getHandle(joint_names_[i]);
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM("JointVelocityExampleController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }
  elapsed_time_ = ros::Duration(0.0);
  return true;
}

void JointVelocityExampleController::update(const ros::Time& /*time*/,
                                            const ros::Duration& period) {
  elapsed_time_ += period;

  ros::Duration time_max(8.0);
  double omega_max = 0.1;
  double cycle = std::floor(
      std::pow(-1.0, (elapsed_time_.toSec() - std::fmod(elapsed_time_.toSec(), time_max.toSec())) /
                         time_max.toSec()));
  double omega = cycle * omega_max / 2.0 *
                 (1.0 - std::cos(2.0 * M_PI / time_max.toSec() * elapsed_time_.toSec()));

  for (auto joint_handle : velocity_joint_handles_) {
    joint_handle.setCommand(omega);
  }
}

void JointVelocityExampleController::stopping(const ros::Time& /*time*/) {
  for (auto joint_handle : velocity_joint_handles_) {
    joint_handle.setCommand(0.0);
  }
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::JointVelocityExampleController,
                       controller_interface::ControllerBase)
