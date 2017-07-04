
#include <franka_example_controllers/cartesian_pose_example_controller.h>

#include <cmath>
#include <stdexcept>
#include <string>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <xmlrpcpp/XmlRpcValue.h>

#include <franka_hw/franka_cartesian_command_interface.h>

namespace franka_example_controllers {

CartesianPoseExampleController::CartesianPoseExampleController()
    : cartesian_pose_interface_(nullptr), elapsed_time_(0.0) {}

bool CartesianPoseExampleController::init(hardware_interface::RobotHW* robot_hw,
                                          ros::NodeHandle& node_handle) {
  cartesian_pose_interface_ =
      robot_hw->get<franka_hw::FrankaPoseCartesianInterface>();
  if (cartesian_pose_interface_ == nullptr) {
    ROS_ERROR("Could not get Cartesian Pose interface from hardware");
    return false;
  }

  if (!node_handle.getParam("/franka_hw_node/arm_id", arm_id_)) {
    ROS_ERROR("Could not get parameter arm_id");
    return false;
  }

  try {
    franka_hw::FrankaCartesianPoseHandle cartesian_pose_handle(
        cartesian_pose_interface_->getHandle(arm_id_ +
                                             std::string("_cartesian")));
    initial_pose_ = cartesian_pose_handle.getRobotState().O_T_EE;
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM("Exception getting cartesian handle: " << e.what());
    return false;
  }
  elapsed_time_ = ros::Duration(0.0);
  return true;
}

void CartesianPoseExampleController::update(const ros::Time& time,  // NOLINT
                                            const ros::Duration& period) {
  double radius = 0.3;
  double angle = M_PI / 4 * (1 - std::cos(M_PI / 5.0 * elapsed_time_.toSec()));
  double delta_x = radius * std::sin(angle);
  double delta_z = radius * (std::cos(angle) - 1);
  std::array<double, 16> new_pose = initial_pose_;
  new_pose[12] += delta_x;
  new_pose[14] += delta_z;
  try {
    franka_hw::FrankaCartesianPoseHandle cartesian_pose_handle(
        cartesian_pose_interface_->getHandle(arm_id_ +
                                             std::string("_cartesian")));
    cartesian_pose_handle.setCommand(new_pose);
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM("Exception getting cartesian handle: " << e.what());
    return;
  }
  elapsed_time_ += period;
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(
    franka_example_controllers::CartesianPoseExampleController,
    controller_interface::ControllerBase)
