// Copyright (c) 2019 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_hw/franka_combinable_hw.h>

#include <hardware_interface/joint_command_interface.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/Bool.h>

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
  FrankaHW::initRobot();
  control_loop_thread_ = std::make_unique<std::thread>(&FrankaCombinableHW::controlLoop, this);
}

void FrankaCombinableHW::publishErrorState(const bool error) {
  std_msgs::Bool msg;
  msg.data = static_cast<int>(error);
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

      {
        std::lock_guard<std::mutex> ros_state_lock(ros_state_mutex_);
        std::lock_guard<std::mutex> libfranka_state_lock(libfranka_state_mutex_);
        robot_state_libfranka_ = robot_->readOnce();
        robot_state_ros_ = robot_->readOnce();
      }

      if (!ros::ok()) {
        return;
      }
    }
    ROS_INFO("FrankaCombinableHW::%s::control_loop(): controller is active.", arm_id_.c_str());

    // Reset commands
    {
      std::lock_guard<std::mutex> command_lock(libfranka_cmd_mutex_);
      effort_joint_command_libfranka_ = franka::Torques({0., 0., 0., 0., 0., 0., 0.});
    }

    try {
      control();
    } catch (const franka::ControlException& e) {
      // Reflex could be caught and it needs to wait for automatic error recovery
      ROS_ERROR("%s: %s", arm_id_.c_str(), e.what());
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

void FrankaCombinableHW::control(  // NOLINT (google-default-arguments)
    const std::function<bool(const ros::Time&, const ros::Duration&)>& /*ros_callback*/) const {
  if (!controller_active_) {
    return;
  }
  auto empty_method = [](const franka::RobotState&, franka::Duration) { return true; };
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

std::string FrankaCombinableHW::getArmID() const noexcept {
  return arm_id_;
}

void FrankaCombinableHW::triggerError() {
  has_error_ = true;
  publishErrorState(has_error_);
}

bool FrankaCombinableHW::hasError() const noexcept {
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

bool FrankaCombinableHW::controllerNeedsReset() const noexcept {
  return controller_needs_reset_;
}

bool FrankaCombinableHW::setRunFunction(const ControlMode& requested_control_mode,
                                        const bool limit_rate,
                                        const double cutoff_frequency,
                                        const franka::ControllerMode /*internal_controller*/) {
  using Callback = std::function<bool(const franka::RobotState&, franka::Duration)>;

  if (requested_control_mode == ControlMode::None) {
    return true;
  }
  if (requested_control_mode == ControlMode::JointTorque) {
    run_function_ = [this, limit_rate, cutoff_frequency](franka::Robot& robot,
                                                         Callback /*callback*/) {
      robot.control(std::bind(&FrankaCombinableHW::libfrankaUpdateCallback<franka::Torques>, this,
                              std::cref(effort_joint_command_libfranka_), std::placeholders::_1,
                              std::placeholders::_2),
                    limit_rate, cutoff_frequency);
    };
    return true;
  }

  ROS_ERROR("FrankaCombinableHW: No valid control mode selected; cannot set run function.");
  return false;
}

}  // namespace franka_hw

PLUGINLIB_EXPORT_CLASS(franka_hw::FrankaCombinableHW, hardware_interface::RobotHW)
