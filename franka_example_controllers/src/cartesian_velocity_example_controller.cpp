
#include <franka_example_controllers/cartesian_velocity_example_controller.h>

#include <cmath>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <xmlrpcpp/XmlRpcValue.h>

namespace franka_example_controllers {

CartesianVelocityExampleController::CartesianVelocityExampleController()
    : velocity_cartesian_interface_(nullptr),
      velocity_cartesian_handle_(nullptr),
      elapsed_time_(0.0) {}

bool CartesianVelocityExampleController::init(
    hardware_interface::RobotHW* robot_hardware,
    ros::NodeHandle& root_node_handle,
    ros::NodeHandle& /*controller_node_handle*/) {
  if (!root_node_handle.getParam("arm_id", arm_id_)) {
    ROS_ERROR(
        "CartesianVelocityExampleController: Could not get parameter arm_id");
    return false;
  }

  velocity_cartesian_interface_ =
      robot_hardware->get<franka_hw::FrankaVelocityCartesianInterface>();
  if (velocity_cartesian_interface_ == nullptr) {
    ROS_ERROR(
        "CartesianVelocityExampleController: Could not get Cartesian Pose "
        "interface from hardware");
    return false;
  }
  try {
    velocity_cartesian_handle_.reset(
        new franka_hw::FrankaCartesianVelocityHandle(
            velocity_cartesian_interface_->getHandle(arm_id_ + "_robot")));
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "CartesianVelocityExampleController: Exception getting cartesian "
        "handle: "
        << e.what());
    return false;
  }

  elapsed_time_ = ros::Duration(0.0);
  return true;
}

void CartesianVelocityExampleController::update(const ros::Time& /*time*/,
                                                const ros::Duration& period) {
  double time_max = 4.0;
  double v_max = 0.1;
  double angle = M_PI / 4.0;
  double cycle =
      std::floor(pow(-1.0, (elapsed_time_.toSec() -
                            std::fmod(elapsed_time_.toSec(), time_max)) /
                               time_max));
  double v = cycle * v_max / 2.0 *
             (1.0 - std::cos(2.0 * M_PI / time_max * elapsed_time_.toSec()));
  double v_x = std::cos(angle) * v;
  double v_z = -std::sin(angle) * v;
  std::array<double, 6> command = {{v_x, 0.0, v_z, 0.0, 0.0, 0.0}};
  velocity_cartesian_handle_->setCommand(command);
  elapsed_time_ += period;
  return;
}

void CartesianVelocityExampleController::stopping(const ros::Time& /*time*/) {
  try {
    franka_hw::FrankaCartesianVelocityHandle cartesian_velocity_handle(
        velocity_cartesian_interface_->getHandle(arm_id_ +
                                                 std::string("_cartesian")));
    std::array<double, 6> command = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    cartesian_velocity_handle.setCommand(command);
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "CartesianVelocityExampleController: Exception getting cartesian "
        "handle: "
        << e.what());
  }
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(
    franka_example_controllers::CartesianVelocityExampleController,
    controller_interface::ControllerBase)
