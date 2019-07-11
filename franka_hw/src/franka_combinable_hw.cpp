
#include <hardware_interface/joint_command_interface.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/Bool.h>

#include <franka_hw/franka_combinable_hw.h>
#include <franka_hw/services.h>

namespace franka_hw {

FrankaCombinableHW::FrankaCombinableHW() : has_error_(false), error_recovered_(false) {}

void FrankaCombinableHW::initROSInterfaces(ros::NodeHandle& robot_hw_nh) {
  setupJointStateInterface(robot_state_ros_);
  setupJointCommandInterface(effort_joint_command_ros_.tau_J, robot_state_ros_, false,
                             effort_joint_interface_);
  setupLimitInterface<joint_limits_interface::EffortJointSoftLimitsHandle>(
      effort_joint_limit_interface_, effort_joint_interface_);
  setupFrankaStateInterface(robot_state_ros_);
  setupFrankaModelInterface(robot_state_ros_);

  has_error_pub_ = robot_hw_nh.advertise<std_msgs::Bool>("has_error", 1, true);
  publishErrorState(has_error_);

  setupServicesAndActionServers(robot_hw_nh);
}

void FrankaCombinableHW::initRobot() {
  robot_ = std::make_unique<franka::Robot>(robot_ip_);
  robot_->setCollisionBehavior(
      {{40.0, 40.0, 38.0, 38.0, 32.0, 30.0, 24.0}}, {{40.0, 40.0, 38.0, 36.0, 32.0, 30.0, 24.0}},
      {{40.0, 40.0, 38.0, 38.0, 32.0, 30.0, 24.0}}, {{40.0, 40.0, 38.0, 36.0, 32.0, 30.0, 24.0}},
      {{40.0, 40.0, 40.0, 50.0, 50.0, 50.0}}, {{40.0, 40.0, 40.0, 50.0, 50.0, 50.0}},
      {{40.0, 40.0, 40.0, 50.0, 50.0, 50.0}}, {{40.0, 40.0, 40.0, 50.0, 50.0, 50.0}});
  update(robot_->readOnce());
  model_ = std::make_unique<franka::Model>(robot_->loadModel());

  control_loop_thread_ = std::make_unique<std::thread>(&FrankaCombinableHW::controlLoop, this);
}

void FrankaCombinableHW::publishErrorState(const bool error) {
  std_msgs::Bool msg;
  msg.data = error;  // NOLINT (readability-implicit-bool-cast)
  has_error_pub_.publish(msg);
}

void FrankaCombinableHW::controlLoop() {
  while (ros::ok()) {
    ros::Time last_time = ros::Time::now();

    // Wait until controller has been activated or error has been recovered
    while (!controllerActive() || has_error_) {
      if (!controllerActive()) {
        ROS_DEBUG_THROTTLE(1, "FrankaCombinableHW::%s::control_loop(): controller is not active.",
                           arm_id_.c_str());
      }
      if (has_error_) {
        ROS_DEBUG_THROTTLE(1, "FrankaCombinableHW::%s::control_loop(): an error has occured.",
                           arm_id_.c_str());
      }

      checkJointLimits();

      ros_state_mutex_.lock();
      libfranka_state_mutex_.lock();
      robot_state_libfranka_ = robot_->readOnce();
      robot_state_ros_ = robot_->readOnce();
      libfranka_state_mutex_.unlock();
      ros_state_mutex_.unlock();
      if (!ros::ok()) {
        return;
      }
    }
    ROS_INFO("FrankaCombinableHW::%s::control_loop(): controller is active.", arm_id_.c_str());

    // Reset commands
    libfranka_cmd_mutex_.lock();
    effort_joint_command_libfranka_ = franka::Torques({0., 0., 0., 0., 0., 0., 0.});
    libfranka_cmd_mutex_.unlock();

    try {
      // Run control loop. It will exit if the controller is switched.
      auto empty_method = [](const ros::Time&, const ros::Duration&) -> bool { return true; };
      control(empty_method);
    } catch (const franka::ControlException& e) {
      // Reflex could be caught and it needs to wait for automatic error recovery
      ROS_ERROR("%s", e.what());
      has_error_ = true;
      publishErrorState(has_error_);
    }
  }
}

void FrankaCombinableHW::setupServicesAndActionServers(ros::NodeHandle& node_handle) {
  setupServices(*robot_, node_handle, services_);
  recovery_action_server_ =
      std::make_unique<actionlib::SimpleActionServer<franka_msgs::ErrorRecoveryAction>>(
          node_handle, "error_recovery",
          [&](const franka_msgs::ErrorRecoveryGoalConstPtr&) {
            try {
              robot_->automaticErrorRecovery();
              // error recovered => reset controller
              if (has_error_) {
                error_recovered_ = true;
              }
              has_error_ = false;
              publishErrorState(has_error_);
              recovery_action_server_->setSucceeded();
            } catch (const franka::Exception& ex) {
              recovery_action_server_->setAborted(franka_msgs::ErrorRecoveryResult(), ex.what());
            }
          },
          false);
  recovery_action_server_->start();
}

void FrankaCombinableHW::control(
    const std::function<bool(const ros::Time&, const ros::Duration&)>& /*ros_callback*/) {
  if (!controller_active_) {
    return;
  }
  auto empty_method = [](const franka::RobotState&, franka::Duration) -> bool { return true; };
  run_function_(*robot_, empty_method);
}

bool FrankaCombinableHW::checkForConflict(
    const std::list<hardware_interface::ControllerInfo>& info) const {
  ResourceWithClaimsMap resource_map = getResourceMap(info);

  if (hasConflictingMultiClaim(resource_map)) {
    return true;
  }

  ArmClaimedMap arm_claim_map;
  if (!getArmClaimedMap(resource_map, arm_claim_map)) {
    ROS_ERROR("FrankaCombinableHW: Unknown interface claimed. Conflict!");
    return true;
  }

  // check for any claim to trajectory interfaces (non-torque) which are not supported.
  if (hasTrajectoryClaim(arm_claim_map, arm_id_)) {
    ROS_ERROR_STREAM("FrankaCombinableHW: Invalid claim joint position or velocity interface."
                     << "Note: joint position and joint velocity interfaces are not supported"
                     << " in FrankaCombinableHW. Arm:" << arm_id_ << ". Conflict!");
    return true;
  }

  return partiallyClaimsArmJoints(arm_claim_map, arm_id_);
}

void FrankaCombinableHW::read(const ros::Time& time, const ros::Duration& period) {
  controller_needs_reset_ = bool(error_recovered_);
  FrankaHW::read(time, period);
}

void FrankaCombinableHW::write(const ros::Time& time, const ros::Duration& period) {
  // if flag `controller_needs_reset_` was updated, then controller_manager. update(...,
  // reset_controller) must
  // have been executed to reset the controller.
  if (controller_needs_reset_ && error_recovered_) {
    controller_needs_reset_ = false;
    error_recovered_ = false;
  }

  enforceLimits(period);

  FrankaHW::write(time, period);
}

std::array<double, 7> FrankaCombinableHW::saturateTorqueRate(
    const std::array<double, 7>& tau_d_calculated,
    const std::array<double, 7>& tau_J_d) {  // NOLINT (readability-identifier-naming)
  const double kDeltaTauMax = 1.0;
  std::array<double, 7> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] = tau_J_d[i] + std::max(std::min(difference, kDeltaTauMax), -kDeltaTauMax);
  }
  return tau_d_saturated;
}

std::string FrankaCombinableHW::getArmID() {
  return arm_id_;
}

void FrankaCombinableHW::triggerError() {
  has_error_ = true;
  publishErrorState(has_error_);
}

bool FrankaCombinableHW::hasError() {
  return has_error_;
}

void FrankaCombinableHW::resetError() {
  robot_->automaticErrorRecovery();
  // error recovered => reset controller
  if (has_error_) {
    error_recovered_ = true;
  }
  has_error_ = false;
  publishErrorState(has_error_);
}

bool FrankaCombinableHW::controllerNeedsReset() {
  return controller_needs_reset_;
}

bool FrankaCombinableHW::setRunFunction(const ControlMode& requested_control_mode,
                                        const bool limit_rate,
                                        const double cutoff_frequency,
                                        const franka::ControllerMode internal_controller) {
  using Callback = std::function<bool(const franka::RobotState&, franka::Duration)>;

  if (requested_control_mode == ControlMode::None) {
    return true;
  }
  if (requested_control_mode == ControlMode::JointTorque) {
    run_function_ = [this, limit_rate](franka::Robot& robot, Callback /*callback*/) {
      robot.control(std::bind(&FrankaCombinableHW::libfrankaUpdateCallback<franka::Torques>, this,
                              std::cref(effort_joint_command_libfranka_), std::placeholders::_1,
                              std::placeholders::_2),
                    limit_rate);
    };
    return true;
  }

  ROS_ERROR("FrankaCombinableHW: No valid control mode selected; cannot set run function.");
  return false;
}

}  // namespace franka_hw

PLUGINLIB_EXPORT_CLASS(franka_hw::FrankaCombinableHW, hardware_interface::RobotHW)
