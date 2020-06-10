// Copyright (c) 2020 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/teleop_joint_pd_example_controller.h>

#include <hardware_interface/hardware_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <functional>
#include <string>
#include <vector>

namespace franka_example_controllers {

bool TeleopJointPDExampleController::init(hardware_interface::RobotHW* robot_hw,
                                          ros::NodeHandle& node_handle) {
  std::string master_arm_id;
  if (!node_handle.getParam("master/arm_id", master_arm_id)) {
    ROS_ERROR("TeleopJointPDExampleController: Could not read parameter master_arm_id");
    return false;
  }
  std::vector<std::string> master_joint_names;
  if (!node_handle.getParam("master/joint_names", master_joint_names) ||
      master_joint_names.size() != 7) {
    ROS_ERROR(
        "TeleopJointPDExampleController: Invalid or no master_joint_names parameters provided, "
        "aborting controller init!");
    return false;
  }
  std::string slave_arm_id;
  if (!node_handle.getParam("slave/arm_id", slave_arm_id)) {
    ROS_ERROR("TeleopJointPDExampleController: Could not read parameter slave_arm_id");
    return false;
  }
  std::vector<std::string> slave_joint_names;
  if (!node_handle.getParam("slave/joint_names", slave_joint_names) ||
      slave_joint_names.size() != 7) {
    ROS_ERROR(
        "TeleopJointPDExampleController: Invalid or no slave_joint_names parameters provided, "
        "aborting controller init!");
    return false;
  }

  std::vector<double> k_d_master;
  if (!node_handle.getParam("master/d_gains", k_d_master) || k_d_master.size() != 7) {
    ROS_ERROR(
        "TeleopJointPDExampleController: Invalid or no master/d_gains provided, aborting "
        "controller init!");
    return false;
  }
  k_d_master_ = Eigen::Map<Vector7d>(k_d_master.data());

  std::vector<double> k_p_slave;
  if (!node_handle.getParam("slave/p_gains", k_p_slave) || k_p_slave.size() != 7) {
    ROS_ERROR(
        "TeleopJointPDExampleController: Invalid or no slave/p_gains provided, aborting "
        "controller init!");
    return false;
  }
  k_p_slave_ = Eigen::Map<Vector7d>(k_p_slave.data());

  std::vector<double> k_d_slave;
  if (!node_handle.getParam("slave/d_gains", k_d_slave) || k_d_slave.size() != 7) {
    ROS_ERROR(
        "TeleopJointPDExampleController: Invalid or no slave/d_gains provided, aborting "
        "controller init!");
    return false;
  }
  k_d_slave_ = Eigen::Map<Vector7d>(k_d_slave.data());

  std::vector<double> k_dq;
  if (!node_handle.getParam("slave/drift_comp_gains", k_dq) || k_dq.size() != 7) {
    ROS_ERROR(
        "TeleopJointPDExampleController: Invalid or no slave/drift_comp_gains provided, aborting "
        "controller init!");
    return false;
  }
  k_dq_ = Eigen::Map<Vector7d>(k_dq.data());

  std::vector<double> dq_max_lower;
  if (!node_handle.getParam("slave/dq_max_lower", dq_max_lower) || dq_max_lower.size() != 7) {
    ROS_ERROR(
        "TeleopJointPDExampleController: Invalid or no slave/dq_max_lower provided, aborting "
        "controller init!");
    return false;
  }
  dq_max_lower_ = Eigen::Map<Vector7d>(dq_max_lower.data());

  std::vector<double> dq_max_upper;
  if (!node_handle.getParam("slave/dq_max_upper", dq_max_upper) || dq_max_upper.size() != 7) {
    ROS_ERROR(
        "TeleopJointPDExampleController: Invalid or no slave/dq_max_upper provided, aborting "
        "controller init!");
    return false;
  }
  dq_max_upper_ = Eigen::Map<Vector7d>(dq_max_upper.data());

  std::vector<double> ddq_max_lower;
  if (!node_handle.getParam("slave/ddq_max_lower", ddq_max_lower) || ddq_max_lower.size() != 7) {
    ROS_ERROR(
        "TeleopJointPDExampleController: Invalid or no slave/ddq_max_lower provided, aborting "
        "controller init!");
    return false;
  }
  ddq_max_lower_ = Eigen::Map<Vector7d>(ddq_max_lower.data());

  std::vector<double> ddq_max_upper;
  if (!node_handle.getParam("slave/ddq_max_upper", ddq_max_upper) || ddq_max_upper.size() != 7) {
    ROS_ERROR(
        "TeleopJointPDExampleController: Invalid or no slave/ddq_max_upper provided, aborting "
        "controller init!");
    return false;
  }
  ddq_max_upper_ = Eigen::Map<Vector7d>(ddq_max_upper.data());

  if (!node_handle.getParam("master/contact_force_threshold",
                            master_data_.contact_force_threshold)) {
    ROS_ERROR(
        "TeleopJointPDExampleController: Invalid or no master/contact_force_threshold provided, "
        "aborting controller init!");
    return false;
  }

  if (!node_handle.getParam("slave/contact_force_threshold", slave_data_.contact_force_threshold)) {
    ROS_ERROR(
        "TeleopJointPDExampleController: Invalid or no slave/contact_force_threshold provided, "
        "aborting controller init!");
    return false;
  }

  debug_ = false;
  if (!node_handle.getParam("debug", debug_)) {
    ROS_INFO_STREAM("TeleopJointPDExampleController: Could not find parameter debug. Defaulting to "
                    << std::boolalpha << debug_);
  }

  // Init for each arm
  if (!initArm(robot_hw, master_data_, master_arm_id, master_joint_names) ||
      !initArm(robot_hw, slave_data_, slave_arm_id, slave_joint_names)) {
    return false;
  }

  if (debug_) {
    // Init for dynamic reconfigure
    dynamic_reconfigure_teleop_param_node_ =
        ros::NodeHandle("dynamic_reconfigure_teleop_param_node");
    dynamic_server_teleop_param_ = std::make_unique<
        dynamic_reconfigure::Server<franka_example_controllers::teleop_paramConfig>>(
        dynamic_reconfigure_teleop_param_node_);
    dynamic_server_teleop_param_->setCallback(
        boost::bind(&TeleopJointPDExampleController::teleopParamCallback, this, _1, _2));

    // Init for publishers
    master_target_pub_.init(node_handle, "master_target", 1);
    master_target_pub_.lock();
    master_target_pub_.msg_.name.resize(7);
    master_target_pub_.msg_.position.resize(7);
    master_target_pub_.msg_.velocity.resize(7);
    master_target_pub_.msg_.effort.resize(7);
    master_target_pub_.unlock();

    slave_target_pub_.init(node_handle, "slave_target", 1);
    slave_target_pub_.lock();
    slave_target_pub_.msg_.name.resize(7);
    slave_target_pub_.msg_.position.resize(7);
    slave_target_pub_.msg_.velocity.resize(7);
    slave_target_pub_.msg_.effort.resize(7);
    slave_target_pub_.unlock();

    master_contact_pub_.init(node_handle, "master_contact", 1);
    slave_contact_pub_.init(node_handle, "slave_contact", 1);
  }

  return true;
}

void TeleopJointPDExampleController::starting(const ros::Time& time) {
  franka::RobotState slave_robot_state = slave_data_.state_handle->getRobotState();
  q_target_last_ = Eigen::Map<Vector7d>(slave_robot_state.q.data());
  dq_target_last_.setZero();
  master_data_.tau_target_last.setZero();
  slave_data_.tau_target_last.setZero();
}

void TeleopJointPDExampleController::update(const ros::Time& time, const ros::Duration& period) {
  franka::RobotState master_robot_state = master_data_.state_handle->getRobotState();
  franka::RobotState slave_robot_state = slave_data_.state_handle->getRobotState();
  master_data_.q = Eigen::Map<Vector7d>(master_robot_state.q.data());
  master_data_.dq = Eigen::Map<Vector7d>(master_robot_state.dq.data());
  slave_data_.q = Eigen::Map<Vector7d>(slave_robot_state.q.data());
  slave_data_.dq = Eigen::Map<Vector7d>(slave_robot_state.dq.data());

  // Determine contact scaling factor depending on the external cartesian forces applied on the
  // endeffector.
  Vector6d master_f_ext_hat = Eigen::Map<Vector6d>(master_robot_state.K_F_ext_hat_K.data());
  master_data_.f_ext_norm = master_f_ext_hat.head(3).norm();
  master_data_.contact =
      rampParameter(master_data_.f_ext_norm, 1.0, 0.0, master_data_.contact_force_threshold,
                    master_data_.contact_ramp_increase);

  Vector6d slave_f_ext_hat = Eigen::Map<Vector6d>(slave_robot_state.K_F_ext_hat_K.data());
  slave_data_.f_ext_norm = slave_f_ext_hat.head(3).norm();
  slave_data_.contact =
      rampParameter(slave_data_.f_ext_norm, 1.0, 0.0, slave_data_.contact_force_threshold,
                    slave_data_.contact_ramp_increase);

  // Determine max velocities and accelerations of slave arm depending on tracking errors to avoid
  // jumps and high velocities when starting example.
  Vector7d q_deviation = (q_target_last_ - master_data_.q).cwiseAbs();
  Vector7d dq_max;
  Vector7d ddq_max;
  for (size_t i = 0; i < 7; ++i) {
    dq_max[i] = rampParameter(q_deviation[i], dq_max_lower_[i], dq_max_upper_[i],
                              velocity_ramp_shift_, velocity_ramp_increase_);
    ddq_max[i] = rampParameter(q_deviation[i], ddq_max_lower_[i], ddq_max_upper_[i],
                               velocity_ramp_shift_, velocity_ramp_increase_);
  }

  // Calculate target postions and velocities for slave arm
  dq_unsaturated_ = k_dq_.asDiagonal() * (master_data_.q - q_target_last_) + master_data_.dq;
  dq_target_ = saturateAndLimit(dq_unsaturated_, dq_target_last_, dq_max, ddq_max, period.toSec());
  dq_target_last_ = dq_target_;
  q_target_ = q_target_last_ + (dq_target_ * period.toSec());
  q_target_last_ = q_target_;

  if (!master_robot_state.current_errors && !slave_robot_state.current_errors) {
    // Compute force-feedback for the master arm to render the haptic interaction of the slave
    // robot. Add a slight damping to reduce vibrations.
    Vector7d slave_tau_ext_hat =
        Eigen::Map<Vector7d>(slave_robot_state.tau_ext_hat_filtered.data());
    Vector7d master_damping_torque =
        master_damping_factor_ * k_d_master_.asDiagonal() * master_data_.dq;
    Vector7d master_force_feedback =
        slave_data_.contact *
        (force_feedback_idle_ +
         master_data_.contact * (force_feedback_guiding_ - force_feedback_idle_)) *
        (-slave_tau_ext_hat);

    master_data_.tau_target = master_force_feedback - master_damping_torque;
    master_data_.tau_target_last = master_data_.tau_target;

    // Compute PD control for the slave arm to track the master's motions.
    // slave_data_.tau_target =
    //   slave_p_gain_factor_ * k_p_slave_.asDiagonal() * (q_target_ - slave_data_.q) +
    //   slave_d_gain_factor_ * k_d_slave_.asDiagonal() * (dq_target_ - slave_data_.dq);
    slave_data_.tau_target =
        slave_p_gain_factor_ * k_p_slave_.asDiagonal() * (q_target_ - slave_data_.q) +
        sqrt(slave_p_gain_factor_) * k_d_slave_.asDiagonal() * (dq_target_ - slave_data_.dq);
    slave_data_.tau_target_last = slave_data_.tau_target;

  } else {
    // Control target torques to zero if any arm is in error state.
    master_data_.tau_target = decrease_factor_ * master_data_.tau_target_last;
    master_data_.tau_target_last = master_data_.tau_target;
    slave_data_.tau_target = decrease_factor_ * slave_data_.tau_target_last;
    slave_data_.tau_target_last = slave_data_.tau_target;
  }

  updateArm(master_data_);
  updateArm(slave_data_);

  if (debug_ && publish_rate_()) {
    publishMasterTarget();
    publishSlaveTarget();
    publishMasterContact();
    publishSlaveContact();
  }
}

bool TeleopJointPDExampleController::initArm(hardware_interface::RobotHW* robot_hw,
                                             FrankaDataContainer& arm_data,
                                             const std::string& arm_id,
                                             const std::vector<std::string>& joint_names) {
  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        "TeleopJointPDExampleController: Error getting effort joint interface from hardware of "
        << arm_id << ".");
    return false;
  }
  arm_data.joint_handles.clear();
  for (size_t i = 0; i < 7; ++i) {
    try {
      arm_data.joint_handles.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& e) {
      ROS_ERROR_STREAM(
          "TeleopJointPDExampleController: Exception getting joint handles: " << e.what());
      return false;
    }
  }

  // Get state interface.
  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM(
        "PandaJointImpedanceControllerImpl: Error getting state interface from hardware");
    return false;
  }
  try {
    arm_data.state_handle = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "PandaJointImpedanceControllerImpl: Exception getting state handle from interface: "
        << ex.what());
    return false;
  }

  return true;
}

void TeleopJointPDExampleController::updateArm(FrankaDataContainer& arm_data) {
  for (size_t i = 0; i < 7; ++i) {
    arm_data.joint_handles[i].setCommand(arm_data.tau_target[i]);
  }
}

Eigen::Matrix<double, 7, 1> TeleopJointPDExampleController::saturateAndLimit(
    const Vector7d& x_calc,
    const Vector7d& x_last,
    const Vector7d& x_max,
    const Vector7d& dx_max,
    const double& delta_t) {
  Vector7d x_limited;
  for (size_t i = 0; i < 7; i++) {
    double delta_x_max = dx_max[i] * delta_t;
    double diff = x_calc[i] - x_last[i];
    double x_saturated = x_last[i] + std::max(std::min(diff, delta_x_max), -delta_x_max);
    x_limited[i] = std::max(std::min(x_saturated, x_max[i]), -x_max[i]);
  }
  return x_limited;
}

double TeleopJointPDExampleController::rampParameter(const double& x,
                                                     const double& neg_x_asymptote,
                                                     const double& pos_x_asymptote,
                                                     const double& shift_along_x,
                                                     const double& increase_factor) {
  double ramp =
      0.5 * (pos_x_asymptote + neg_x_asymptote -
             (pos_x_asymptote - neg_x_asymptote) * tanh(increase_factor * (x - shift_along_x)));
  return ramp;
}

void TeleopJointPDExampleController::teleopParamCallback(
    franka_example_controllers::teleop_paramConfig& config,
    uint32_t level) {
  if (dynamic_reconfigure_mutex_.try_lock()) {
    master_damping_factor_ = config.master_damping_factor;
    slave_p_gain_factor_ = config.slave_p_gain_factor;
    slave_d_gain_factor_ = config.slave_d_gain_factor;
    force_feedback_guiding_ = config.force_feedback_guiding;
    force_feedback_idle_ = config.force_feedback_idle;
    slave_data_.contact_force_threshold = config.slave_contact_force_threshold;
    master_data_.contact_force_threshold = config.master_contact_force_threshold;

    /// TODO(puxb_st): Think about removing these. Remove dynamic reconfigure from params in config
    /// file
    dq_max_lower_[0] = config.dq_l_1;
    dq_max_lower_[1] = config.dq_l_2;
    dq_max_lower_[2] = config.dq_l_3;
    dq_max_lower_[3] = config.dq_l_4;
    dq_max_lower_[4] = config.dq_l_5;
    dq_max_lower_[5] = config.dq_l_6;
    dq_max_lower_[6] = config.dq_l_7;

    dq_max_upper_[0] = config.dq_u_1;
    dq_max_upper_[1] = config.dq_u_2;
    dq_max_upper_[2] = config.dq_u_3;
    dq_max_upper_[3] = config.dq_u_4;
    dq_max_upper_[4] = config.dq_u_5;
    dq_max_upper_[5] = config.dq_u_6;
    dq_max_upper_[6] = config.dq_u_7;

    ddq_max_lower_[0] = config.ddq_l_1;
    ddq_max_lower_[1] = config.ddq_l_2;
    ddq_max_lower_[2] = config.ddq_l_3;
    ddq_max_lower_[3] = config.ddq_l_4;
    ddq_max_lower_[4] = config.ddq_l_5;
    ddq_max_lower_[5] = config.ddq_l_6;
    ddq_max_lower_[6] = config.ddq_l_7;

    ddq_max_upper_[0] = config.ddq_u_1;
    ddq_max_upper_[1] = config.ddq_u_2;
    ddq_max_upper_[2] = config.ddq_u_3;
    ddq_max_upper_[3] = config.ddq_u_4;
    ddq_max_upper_[4] = config.ddq_u_5;
    ddq_max_upper_[5] = config.ddq_u_6;
    ddq_max_upper_[6] = config.ddq_u_7;

    ROS_INFO("Dynamic reconfigure: New params set.");
  }
  dynamic_reconfigure_mutex_.unlock();
}

void TeleopJointPDExampleController::publishMasterTarget() {
  if (master_target_pub_.trylock()) {
    for (size_t i = 0; i < 7; ++i) {
      master_target_pub_.msg_.name[i] = "panda_joint" + std::to_string(i + 1);
      master_target_pub_.msg_.position[i] = 0.0;
      master_target_pub_.msg_.velocity[i] = 0.0;
      master_target_pub_.msg_.effort[i] = master_data_.tau_target[i];
    }
    master_target_pub_.unlockAndPublish();
  }
}

void TeleopJointPDExampleController::publishSlaveTarget() {
  if (slave_target_pub_.trylock()) {
    for (size_t i = 0; i < 7; ++i) {
      slave_target_pub_.msg_.name[i] = "panda_joint" + std::to_string(i + 1);
      slave_target_pub_.msg_.position[i] = q_target_[i];
      slave_target_pub_.msg_.velocity[i] = dq_target_[i];
      slave_target_pub_.msg_.effort[i] = slave_data_.tau_target[i];
    }
    slave_target_pub_.unlockAndPublish();
  }
}

void TeleopJointPDExampleController::publishMasterContact() {
  if (master_contact_pub_.trylock()) {
    master_contact_pub_.msg_.data = master_data_.contact;
    master_contact_pub_.unlockAndPublish();
  }
}

void TeleopJointPDExampleController::publishSlaveContact() {
  if (slave_contact_pub_.trylock()) {
    slave_contact_pub_.msg_.data = slave_data_.contact;
    slave_contact_pub_.unlockAndPublish();
  }
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::TeleopJointPDExampleController,
                       controller_interface::ControllerBase)
