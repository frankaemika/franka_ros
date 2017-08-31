#include <franka_example_controllers/force_example_controller.h>

#include <cmath>

#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka/robot_state.h>

namespace franka_example_controllers {

ForceExampleController::ForceExampleController() {}

bool ForceExampleController::init(hardware_interface::RobotHW* robot_hw,
                                           ros::NodeHandle& node_handle) {
  std::string arm_id;
  std::vector<std::string> joint_names;

  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("ForceExampleController: Could not read parameter arm_id");
    return false;
  }

  franka_hw::FrankaModelInterface* model_interface =
      robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "ForceExampleController: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_.reset(
        new franka_hw::FrankaModelHandle(model_interface->getHandle(arm_id + "_model")));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "ForceExampleController: Exception getting model handle from interface: "
        << ex.what());
    return false;
  }

  franka_hw::FrankaStateInterface* state_interface =
      robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM(
        "JointImpedanceExampleController: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_.reset(new franka_hw::FrankaStateHandle(
        state_interface->getHandle(arm_id + "_robot")));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "JointImpedanceExampleController: Exception getting state handle from interface: "
            << ex.what());
    return false;
  }

  hardware_interface::EffortJointInterface* effort_joint_interface =
      robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        "ForceExampleController: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "ForceExampleController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  dynamic_reconfigure_desired_mass_param_node_ = ros::NodeHandle("dynamic_reconfigure_desired_mass_param_node");
  dynamic_server_desired_mass_param_.reset(
      new dynamic_reconfigure::Server<
          franka_example_controllers::desired_mass_paramConfig>(dynamic_reconfigure_desired_mass_param_node_));
  dynamic_server_desired_mass_param_->setCallback(
      boost::bind(&ForceExampleController::desired_mass_param_callback, this, _1, _2));

  return true;
}

void ForceExampleController::starting(const ros::Time& /*time*/) {
}

void ForceExampleController::update(const ros::Time& /*time*/,
                                             const ros::Duration& period) {

  franka::RobotState robot_state = state_handle_->getRobotState();
  std::array<double, 42> jacobian_array = model_handle_->getZeroJacobian(
      franka::Frame::kEndEffector, robot_state);
  Eigen::Map<Eigen::Matrix<double, 6, 7> > jacobian(jacobian_array.data());

  Eigen::VectorXd tau_d(7), desired_force_torque(6);
  desired_force_torque.setZero();
  desired_force_torque(2) = desired_mass_ * -9.81;
  tau_d = jacobian.transpose() * desired_force_torque;

  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_d(i));
  }
  return;
}

void ForceExampleController::desired_mass_param_callback(
    franka_example_controllers::desired_mass_paramConfig& config, uint32_t level) {
  desired_mass_ = config.desired_mass;
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::ForceExampleController,
                       controller_interface::ControllerBase)
