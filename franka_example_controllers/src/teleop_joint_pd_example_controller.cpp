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
  auto get_joint_params = [&node_handle](const std::string& key, auto& vec) {
    if (!node_handle.getParam(key, vec) || vec.size() != 7) {
      ROS_ERROR(
          "TeleopJointPDExampleController: Invalid or no parameter %s provided, "
          "aborting controller init!",
          key.c_str());
      return false;
    }
    return true;
  };

  std::string leader_arm_id;
  if (!node_handle.getParam("leader/arm_id", leader_arm_id)) {
    ROS_ERROR(
        "TeleopJointPDExampleController: Could not read parameter leader_arm_id, aborting "
        "controller init!");
    return false;
  }

  std::vector<std::string> leader_joint_names;
  if (!get_joint_params("leader/joint_names", leader_joint_names)) {
    return false;
  }

  std::string follower_arm_id;
  if (!node_handle.getParam("follower/arm_id", follower_arm_id)) {
    ROS_ERROR(
        "TeleopJointPDExampleController: Could not read parameter follower_arm_id, aborting "
        "controller init!");
    return false;
  }

  std::vector<std::string> follower_joint_names;
  if (!get_joint_params("follower/joint_names", follower_joint_names)) {
    return false;
  }

  std::vector<double> k_d_leader;
  if (!get_joint_params("leader/d_gains", k_d_leader)) {
    return false;
  }
  k_d_leader_ = Eigen::Map<Vector7d>(k_d_leader.data());

  std::vector<double> k_p_follower;
  if (!get_joint_params("follower/p_gains", k_p_follower)) {
    return false;
  }
  k_p_follower_ = Eigen::Map<Vector7d>(k_p_follower.data());

  std::vector<double> k_d_follower;
  if (!get_joint_params("follower/d_gains", k_d_follower)) {
    return false;
  }
  k_d_follower_ = Eigen::Map<Vector7d>(k_d_follower.data());

  std::vector<double> k_dq;
  if (!get_joint_params("follower/drift_comp_gains", k_dq)) {
    return false;
  }
  k_dq_ = Eigen::Map<Vector7d>(k_dq.data());

  std::vector<double> dq_max_lower;
  if (!get_joint_params("follower/dq_max_lower", dq_max_lower)) {
    return false;
  }
  dq_max_lower_ = Eigen::Map<Vector7d>(dq_max_lower.data());

  std::vector<double> dq_max_upper;
  if (!get_joint_params("follower/dq_max_upper", dq_max_upper)) {
    return false;
  }
  dq_max_upper_ = Eigen::Map<Vector7d>(dq_max_upper.data());

  std::vector<double> ddq_max_lower;
  if (!get_joint_params("follower/ddq_max_lower", ddq_max_lower)) {
    return false;
  }
  ddq_max_lower_ = Eigen::Map<Vector7d>(ddq_max_lower.data());

  std::vector<double> ddq_max_upper;
  if (!get_joint_params("follower/ddq_max_upper", ddq_max_upper)) {
    return false;
  }
  ddq_max_upper_ = Eigen::Map<Vector7d>(ddq_max_upper.data());

  if (!node_handle.getParam("leader/contact_force_threshold",
                            leader_data_.contact_force_threshold)) {
    ROS_ERROR(
        "TeleopJointPDExampleController: Invalid or no leader/contact_force_threshold provided, "
        "aborting controller init!");
    return false;
  }

  if (!node_handle.getParam("follower/contact_force_threshold",
                            follower_data_.contact_force_threshold)) {
    ROS_ERROR(
        "TeleopJointPDExampleController: Invalid or no follower/contact_force_threshold provided, "
        "aborting controller init!");
    return false;
  }

  debug_ = false;
  if (!node_handle.getParam("debug", debug_)) {
    ROS_INFO_STREAM("TeleopJointPDExampleController: Could not find parameter debug. Defaulting to "
                    << std::boolalpha << debug_);
  }

  // Init for each arm
  if (!initArm(robot_hw, leader_data_, leader_arm_id, leader_joint_names) ||
      !initArm(robot_hw, follower_data_, follower_arm_id, follower_joint_names)) {
    return false;
  }

  if (debug_) {
    // Init for dynamic reconfigure
    dynamic_reconfigure_teleop_param_node_ = ros::NodeHandle("dyn_reconf_teleop_param_node");
    dynamic_server_teleop_param_ = std::make_unique<
        dynamic_reconfigure::Server<franka_example_controllers::teleop_paramConfig>>(
        dynamic_reconfigure_teleop_param_node_);
    dynamic_server_teleop_param_->setCallback(
        boost::bind(&TeleopJointPDExampleController::teleopParamCallback, this, _1, _2));

    // Init for publishers
    auto init_publisher = [&node_handle](auto& publisher, const auto& topic) {
      publisher.init(node_handle, topic, 1);
      publisher.lock();
      publisher.msg_.name.resize(7);
      publisher.msg_.position.resize(7);
      publisher.msg_.velocity.resize(7);
      publisher.msg_.effort.resize(7);
      publisher.unlock();
    };

    init_publisher(leader_target_pub_, "leader_target");
    init_publisher(follower_target_pub_, "follower_target");
    leader_contact_pub_.init(node_handle, "leader_contact", 1);
    follower_contact_pub_.init(node_handle, "follower_contact", 1);
  }

  return true;
}

void TeleopJointPDExampleController::starting(const ros::Time& /*time*/) {
  franka::RobotState follower_robot_state = follower_data_.state_handle->getRobotState();
  q_target_last_ = Eigen::Map<Vector7d>(follower_robot_state.q.data());
  dq_target_last_.setZero();
  leader_data_.tau_target_last.setZero();
  follower_data_.tau_target_last.setZero();
}

void TeleopJointPDExampleController::update(const ros::Time& /*time*/,
                                            const ros::Duration& period) {
  franka::RobotState leader_robot_state = leader_data_.state_handle->getRobotState();
  franka::RobotState follower_robot_state = follower_data_.state_handle->getRobotState();
  leader_data_.q = Eigen::Map<Vector7d>(leader_robot_state.q.data());
  leader_data_.dq = Eigen::Map<Vector7d>(leader_robot_state.dq.data());
  follower_data_.q = Eigen::Map<Vector7d>(follower_robot_state.q.data());
  follower_data_.dq = Eigen::Map<Vector7d>(follower_robot_state.dq.data());

  // Determine contact scaling factor depending on the external cartesian forces applied on the
  // endeffector.
  Vector6d leader_f_ext_hat = Eigen::Map<Vector6d>(leader_robot_state.K_F_ext_hat_K.data());
  leader_data_.f_ext_norm = leader_f_ext_hat.head(3).norm();
  leader_data_.contact =
      rampParameter(leader_data_.f_ext_norm, 1.0, 0.0, leader_data_.contact_force_threshold,
                    leader_data_.contact_ramp_increase);

  Vector6d follower_f_ext_hat = Eigen::Map<Vector6d>(follower_robot_state.K_F_ext_hat_K.data());
  follower_data_.f_ext_norm = follower_f_ext_hat.head(3).norm();
  follower_data_.contact =
      rampParameter(follower_data_.f_ext_norm, 1.0, 0.0, follower_data_.contact_force_threshold,
                    follower_data_.contact_ramp_increase);

  // Determine max velocities and accelerations of follower arm depending on tracking errors to
  // avoid jumps and high velocities when starting example.
  Vector7d q_deviation = (q_target_last_ - leader_data_.q).cwiseAbs();
  Vector7d dq_max;
  Vector7d ddq_max;
  for (size_t i = 0; i < 7; ++i) {
    dq_max[i] = rampParameter(q_deviation[i], dq_max_lower_[i], dq_max_upper_[i],
                              velocity_ramp_shift_, velocity_ramp_increase_);
    ddq_max[i] = rampParameter(q_deviation[i], ddq_max_lower_[i], ddq_max_upper_[i],
                               velocity_ramp_shift_, velocity_ramp_increase_);
  }

  // Calculate target postions and velocities for follower arm
  dq_unsaturated_ = k_dq_.asDiagonal() * (leader_data_.q - q_target_last_) + leader_data_.dq;
  dq_target_ = saturateAndLimit(dq_unsaturated_, dq_target_last_, dq_max, ddq_max, period.toSec());
  dq_target_last_ = dq_target_;
  q_target_ = q_target_last_ + (dq_target_ * period.toSec());
  q_target_last_ = q_target_;

  if (!leader_robot_state.current_errors && !follower_robot_state.current_errors) {
    // Compute force-feedback for the leader arm to render the haptic interaction of the follower
    // robot. Add a slight damping to reduce vibrations.
    // The force feedback is applied when the external forces on the follower arm exceed a
    // threshold. While the leader arm is unguided (not in contact), the force-feedback is reduced.
    Vector7d follower_tau_ext_hat =
        Eigen::Map<Vector7d>(follower_robot_state.tau_ext_hat_filtered.data());
    Vector7d leader_damping_torque =
        leader_damping_scaling_ * k_d_leader_.asDiagonal() * leader_data_.dq;
    Vector7d leader_force_feedback =
        follower_data_.contact *
        (force_feedback_idle_ +
         leader_data_.contact * (force_feedback_guiding_ - force_feedback_idle_)) *
        (-follower_tau_ext_hat);

    leader_data_.tau_target = leader_force_feedback - leader_damping_torque;
    leader_data_.tau_target_last = leader_data_.tau_target;

    // Compute PD control for the follower arm to track the leader's motions.
    follower_data_.tau_target =
        follower_stiffness_scaling_ * k_p_follower_.asDiagonal() * (q_target_ - follower_data_.q) +
        sqrt(follower_stiffness_scaling_) * k_d_follower_.asDiagonal() *
            (dq_target_ - follower_data_.dq);
    follower_data_.tau_target_last = follower_data_.tau_target;

  } else {
    // Control target torques to zero if any arm is in error state.
    leader_data_.tau_target = decrease_factor_ * leader_data_.tau_target_last;
    leader_data_.tau_target_last = leader_data_.tau_target;
    follower_data_.tau_target = decrease_factor_ * follower_data_.tau_target_last;
    follower_data_.tau_target_last = follower_data_.tau_target;
  }

  updateArm(leader_data_);
  updateArm(follower_data_);

  if (debug_ && publish_rate_()) {
    publishLeaderTarget();
    publishFollowerTarget();
    publishLeaderContact();
    publishFollowerContact();
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
    ROS_ERROR_STREAM("TeleopJointPDExampleController: Error getting state interface from hardware");
    return false;
  }
  try {
    arm_data.state_handle = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "TeleopJointPDExampleController: Exception getting state handle from interface: "
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
    uint32_t /*level*/) {
  if (dynamic_reconfigure_mutex_.try_lock()) {
    leader_damping_scaling_ = config.leader_damping_scaling;
    follower_stiffness_scaling_ = config.follower_stiffness_scaling;
    force_feedback_guiding_ = config.force_feedback_guiding;
    force_feedback_idle_ = config.force_feedback_idle;
    follower_data_.contact_force_threshold = config.follower_contact_force_threshold;
    leader_data_.contact_force_threshold = config.leader_contact_force_threshold;

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

    ROS_INFO("Dynamic reconfigure: Controller params set.");
  }
  dynamic_reconfigure_mutex_.unlock();
}

void TeleopJointPDExampleController::publishLeaderTarget() {
  if (leader_target_pub_.trylock()) {
    for (size_t i = 0; i < 7; ++i) {
      leader_target_pub_.msg_.name[i] = "panda_joint" + std::to_string(i + 1);
      leader_target_pub_.msg_.position[i] = 0.0;
      leader_target_pub_.msg_.velocity[i] = 0.0;
      leader_target_pub_.msg_.effort[i] = leader_data_.tau_target[i];
    }
    leader_target_pub_.unlockAndPublish();
  }
}

void TeleopJointPDExampleController::publishFollowerTarget() {
  if (follower_target_pub_.trylock()) {
    for (size_t i = 0; i < 7; ++i) {
      follower_target_pub_.msg_.name[i] = "panda_joint" + std::to_string(i + 1);
      follower_target_pub_.msg_.position[i] = q_target_[i];
      follower_target_pub_.msg_.velocity[i] = dq_target_[i];
      follower_target_pub_.msg_.effort[i] = follower_data_.tau_target[i];
    }
    follower_target_pub_.unlockAndPublish();
  }
}

void TeleopJointPDExampleController::publishLeaderContact() {
  if (leader_contact_pub_.trylock()) {
    leader_contact_pub_.msg_.data = leader_data_.contact;
    leader_contact_pub_.unlockAndPublish();
  }
}

void TeleopJointPDExampleController::publishFollowerContact() {
  if (follower_contact_pub_.trylock()) {
    follower_contact_pub_.msg_.data = follower_data_.contact;
    follower_contact_pub_.unlockAndPublish();
  }
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::TeleopJointPDExampleController,
                       controller_interface::ControllerBase)
