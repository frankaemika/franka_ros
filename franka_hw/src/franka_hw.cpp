// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_hw/franka_hw.h>
#include <franka_hw/resource_helpers.h>

#include <array>
#include <cstdint>
#include <functional>
#include <list>
#include <ostream>
#include <string>
#include <utility>

#include <franka/control_types.h>
#include <franka/rate_limiting.h>
#include <franka/robot.h>
#include <joint_limits_interface/joint_limits_urdf.h>

namespace franka_hw {

using std::string;
using std::to_string;
using std::function;
using std::array;
using std::list;
using std::placeholders::_1;
using std::placeholders::_2;
using franka::RobotState;
using franka::Robot;
using franka::Torques;
using franka::JointPositions;
using franka::JointVelocities;
using franka::CartesianPose;
using franka::CartesianVelocities;
using hardware_interface::JointStateHandle;
using hardware_interface::JointHandle;
using hardware_interface::HardwareInterfaceException;
using hardware_interface::ControllerInfo;
using joint_limits_interface::PositionJointSoftLimitsHandle;
using joint_limits_interface::VelocityJointSoftLimitsHandle;
using joint_limits_interface::EffortJointSoftLimitsHandle;
using joint_limits_interface::JointLimits;
using joint_limits_interface::getJointLimits;
using Callback = std::function<bool(const franka::RobotState&, franka::Duration)>;

namespace {
std::ostream& operator<<(std::ostream& ostream, franka::ControllerMode mode) {
  if (mode == franka::ControllerMode::kJointImpedance) {
    ostream << "joint_impedance";
  } else if (mode == franka::ControllerMode::kCartesianImpedance) {
    ostream << "cartesian_impedance";
  } else {
    ostream << "<unknown>";
  }
  return ostream;
}
}  // anonymous namespace

FrankaHW::FrankaHW(const array<string, 7>& joint_names,
                   const string& arm_id,
                   const urdf::Model& urdf_model,
                   function<bool()> get_limit_rate,
                   function<double()> get_cutoff_frequency,
                   function<franka::ControllerMode()> get_internal_controller)
    : joint_names_(joint_names),
      arm_id_(arm_id),
      get_internal_controller_(std::move(get_internal_controller)),
      get_limit_rate_(std::move(get_limit_rate)),
      get_cutoff_frequency_(std::move(get_cutoff_frequency)),
      position_joint_command_({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}),
      velocity_joint_command_({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}),
      effort_joint_command_({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}),
      pose_cartesian_command_(
          {1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0}),
      velocity_cartesian_command_({0.0, 0.0, 0.0, 0.0, 0.0, 0.0}),
      urdf_model_(urdf_model) {
  init();
}

FrankaHW::FrankaHW(const array<string, 7>& joint_names,
                   const string& arm_id,
                   const urdf::Model& urdf_model,
                   franka::Model& model,
                   function<bool()> get_limit_rate,
                   function<double()> get_cutoff_frequency,
                   function<franka::ControllerMode()> get_internal_controller)
    : FrankaHW(joint_names,
               arm_id,
               urdf_model,
               std::move(get_limit_rate),
               std::move(get_cutoff_frequency),
               std::move(get_internal_controller)) {
  setupFrankaModelInterface(model, robot_state_);
}

void FrankaHW::update(const RobotState& robot_state) {
  robot_state_ = robot_state;
}

bool FrankaHW::controllerActive() const noexcept {
  return controller_active_;
}

void FrankaHW::control(Robot& robot,
                       const function<bool(const ros::Time&, const ros::Duration&)>& ros_callback) {
  if (!controller_active_) {
    return;
  }

  franka::Duration last_time = robot_state_.time;

  run_function_(robot, [this, ros_callback, &last_time](const RobotState& robot_state,
                                                        franka::Duration time_step) {
    if (last_time != robot_state.time) {
      last_time = robot_state.time;
      return ros_callback(ros::Time::now(), ros::Duration(time_step.toSec()));
    }
    return true;
  });
}

void FrankaHW::enforceLimits(const ros::Duration& period) {
  if (period.toSec() > 0.0) {
    position_joint_limit_interface_.enforceLimits(period);
    velocity_joint_limit_interface_.enforceLimits(period);
    effort_joint_limit_interface_.enforceLimits(period);
  }
}

bool FrankaHW::checkForConflict(const list<ControllerInfo>& info) const {
  ResourceWithClaimsMap resource_map = getResourceMap(info);
  if (hasConflictingMultiClaim(resource_map)) {
    return true;
  }
  ArmClaimedMap arm_claim_map;
  if (!getArmClaimedMap(resource_map, arm_claim_map)) {
    ROS_ERROR_STREAM("FrankaHW: Unknown interface claimed. Conflict!");
    return true;
  }
  return hasConflictingJointAndCartesianClaim(arm_claim_map, arm_id_) ||
         partiallyClaimsArmJoints(arm_claim_map, arm_id_);
}

// doSwitch runs on the main realtime thread.
void FrankaHW::doSwitch(const list<ControllerInfo>& /* start_list */,
                        const list<ControllerInfo>& /* stop_list */) {
  if (current_control_mode_ != ControlMode::None) {
    reset();
    controller_active_ = true;
  }
}

// prepareSwitch runs on the background message handling thread.
bool FrankaHW::prepareSwitch(const list<ControllerInfo>& start_list,
                             const list<ControllerInfo>& stop_list) {
  ResourceWithClaimsMap start_resource_map = getResourceMap(start_list);
  ArmClaimedMap start_arm_claim_map;
  if (!getArmClaimedMap(start_resource_map, start_arm_claim_map)) {
    ROS_ERROR("FrankaHW: Unknown interface claimed for starting!");
    return false;
  }
  ControlMode start_control_mode = getControlMode(arm_id_, start_arm_claim_map);

  ResourceWithClaimsMap stop_resource_map = getResourceMap(stop_list);
  ArmClaimedMap stop_arm_claim_map;
  if (!getArmClaimedMap(stop_resource_map, stop_arm_claim_map)) {
    ROS_ERROR("FrankaHW: Unknown interface claimed for stopping!");
    return false;
  }
  ControlMode stop_control_mode = getControlMode(arm_id_, stop_arm_claim_map);

  ControlMode requested_control_mode = current_control_mode_;
  requested_control_mode &= ~stop_control_mode;
  requested_control_mode |= start_control_mode;

  if (!setRunFunction(requested_control_mode, get_limit_rate_(), get_cutoff_frequency_(),
                      get_internal_controller_())) {
    return false;
  }
  if (current_control_mode_ != requested_control_mode) {
    ROS_INFO_STREAM("FrankaHW: Prepared switching controllers to "
                    << requested_control_mode << " with parameters "
                    << "limit_rate=" << get_limit_rate_()
                    << ", cutoff_frequency=" << get_cutoff_frequency_()
                    << ", internal_controller=" << get_internal_controller_());
    current_control_mode_ = requested_control_mode;
    controller_active_ = false;
  }

  return true;
}

array<double, 7> FrankaHW::getJointPositionCommand() const noexcept {
  return position_joint_command_.q;
}

array<double, 7> FrankaHW::getJointVelocityCommand() const noexcept {
  return velocity_joint_command_.dq;
}

array<double, 7> FrankaHW::getJointEffortCommand() const noexcept {
  return effort_joint_command_.tau_J;
}

void FrankaHW::reset() {
  position_joint_limit_interface_.reset();
}

void FrankaHW::checkJointLimits() {
  string joint_limits_warning;
  for (const auto& k_joint_name : joint_names_) {
    try {
      auto joint_handle = joint_state_interface_.getHandle(k_joint_name);
      auto urdf_joint = urdf_model_.getJoint(k_joint_name);
      JointLimits joint_limits;
      if (getJointLimits(urdf_joint, joint_limits)) {
        double joint_lower = joint_limits.min_position;
        double joint_upper = joint_limits.max_position;
        double joint_position = joint_handle.getPosition();
        double dist = fmin(fabs(joint_position - joint_lower), fabs(joint_position - joint_upper));
        if (dist < joint_limit_warning_threshold_) {
          joint_limits_warning += "\n\t" + k_joint_name + ": " + to_string(dist * 180 / 3.14) +
                                  " degrees to joint limits (limits: [" + to_string(joint_lower) +
                                  ", " + to_string(joint_upper) + "]" +
                                  " q: " + to_string(joint_position) + ") ";
        }
      } else {
        ROS_ERROR_STREAM_ONCE("FrankaHW: Could not parse joint limit for joint "
                              << k_joint_name << " for joint limit interfaces");
      }
    } catch (const HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM_ONCE("FrankaHW: Could not get joint handle " << k_joint_name << " .\n"
                                                                    << ex.what());
      return;
    }
  }
  if (!joint_limits_warning.empty()) {
    ROS_WARN_THROTTLE(5, "FrankaHW: %s", joint_limits_warning.c_str());
  }
}

void FrankaHW::setupJointStateInterface(RobotState& robot_state) {
  for (size_t i = 0; i < joint_names_.size(); i++) {
    JointStateHandle joint_handle_q(joint_names_[i], &robot_state.q[i], &robot_state.dq[i],
                                    &robot_state.tau_J[i]);
    joint_state_interface_.registerHandle(joint_handle_q);
  }
  registerInterface(&joint_state_interface_);
}

void FrankaHW::setupFrankaStateInterface(RobotState& robot_state) {
  FrankaStateHandle franka_state_handle(arm_id_ + "_robot", robot_state_);
  franka_state_interface_.registerHandle(franka_state_handle);
  registerInterface(&franka_state_interface_);
}

void FrankaHW::setupFrankaCartesianPoseInterface(CartesianPose& pose_cartesian_command) {
  FrankaCartesianPoseHandle franka_cartesian_pose_handle(
      franka_state_interface_.getHandle(arm_id_ + "_robot"), pose_cartesian_command.O_T_EE,
      pose_cartesian_command.elbow);
  franka_pose_cartesian_interface_.registerHandle(franka_cartesian_pose_handle);
  registerInterface(&franka_pose_cartesian_interface_);
}

void FrankaHW::setupFrankaCartesianVelocityInterface(
    franka::CartesianVelocities& velocity_cartesian_command) {
  FrankaCartesianVelocityHandle franka_cartesian_velocity_handle(
      franka_state_interface_.getHandle(arm_id_ + "_robot"), velocity_cartesian_command.O_dP_EE,
      velocity_cartesian_command.elbow);
  franka_velocity_cartesian_interface_.registerHandle(franka_cartesian_velocity_handle);
  registerInterface(&franka_velocity_cartesian_interface_);
}

void FrankaHW::setupFrankaModelInterface(franka::Model& model, RobotState& robot_state) {
  franka_hw::FrankaModelHandle model_handle(arm_id_ + "_model", model, robot_state);
  franka_model_interface_.registerHandle(model_handle);
  registerInterface(&franka_model_interface_);
}

bool FrankaHW::setRunFunction(const ControlMode& requested_control_mode,
                              const bool limit_rate,
                              const double cutoff_frequency,
                              const franka::ControllerMode internal_controller) {
  switch (requested_control_mode) {
    case ControlMode::None:
      break;
    case ControlMode::JointTorque:
      run_function_ = [=](Robot& robot, Callback ros_callback) {
        robot.control(std::bind(&FrankaHW::controlCallback<Torques>, this,
                                std::cref(effort_joint_command_), ros_callback, _1, _2),
                      limit_rate, cutoff_frequency);
      };
      break;
    case ControlMode::JointPosition:
      run_function_ = [=](Robot& robot, Callback ros_callback) {
        robot.control(std::bind(&FrankaHW::controlCallback<JointPositions>, this,
                                std::cref(position_joint_command_), ros_callback, _1, _2),
                      internal_controller, limit_rate, cutoff_frequency);
      };
      break;
    case ControlMode::JointVelocity:
      run_function_ = [=](Robot& robot, Callback ros_callback) {
        robot.control(std::bind(&FrankaHW::controlCallback<JointVelocities>, this,
                                std::cref(velocity_joint_command_), ros_callback, _1, _2),
                      internal_controller, limit_rate, cutoff_frequency);
      };
      break;
    case ControlMode::CartesianPose:
      run_function_ = [=](Robot& robot, Callback ros_callback) {
        robot.control(std::bind(&FrankaHW::controlCallback<CartesianPose>, this,
                                std::cref(pose_cartesian_command_), ros_callback, _1, _2),
                      internal_controller, limit_rate, cutoff_frequency);
      };
      break;
    case ControlMode::CartesianVelocity:
      run_function_ = [=](Robot& robot, Callback ros_callback) {
        robot.control(std::bind(&FrankaHW::controlCallback<CartesianVelocities>, this,
                                std::cref(velocity_cartesian_command_), ros_callback, _1, _2),
                      internal_controller, limit_rate, cutoff_frequency);
      };
      break;
    case (ControlMode::JointTorque | ControlMode::JointPosition):
      run_function_ = [=](Robot& robot, Callback ros_callback) {
        robot.control(std::bind(&FrankaHW::controlCallback<Torques>, this,
                                std::cref(effort_joint_command_), ros_callback, _1, _2),
                      std::bind(&FrankaHW::controlCallback<JointPositions>, this,
                                std::cref(position_joint_command_), ros_callback, _1, _2),
                      limit_rate, cutoff_frequency);
      };
      break;
    case (ControlMode::JointTorque | ControlMode::JointVelocity):
      run_function_ = [=](Robot& robot, Callback ros_callback) {
        robot.control(std::bind(&FrankaHW::controlCallback<Torques>, this,
                                std::cref(effort_joint_command_), ros_callback, _1, _2),
                      std::bind(&FrankaHW::controlCallback<JointVelocities>, this,
                                std::cref(velocity_joint_command_), ros_callback, _1, _2),
                      limit_rate, cutoff_frequency);
      };
      break;
    case (ControlMode::JointTorque | ControlMode::CartesianPose):
      run_function_ = [=](Robot& robot, Callback ros_callback) {
        robot.control(std::bind(&FrankaHW::controlCallback<Torques>, this,
                                std::cref(effort_joint_command_), ros_callback, _1, _2),
                      std::bind(&FrankaHW::controlCallback<CartesianPose>, this,
                                std::cref(pose_cartesian_command_), ros_callback, _1, _2),
                      limit_rate, cutoff_frequency);
      };
      break;
    case (ControlMode::JointTorque | ControlMode::CartesianVelocity):
      run_function_ = [=](Robot& robot, Callback ros_callback) {
        robot.control(std::bind(&FrankaHW::controlCallback<Torques>, this,
                                std::cref(effort_joint_command_), ros_callback, _1, _2),
                      std::bind(&FrankaHW::controlCallback<CartesianVelocities>, this,
                                std::cref(velocity_cartesian_command_), ros_callback, _1, _2),
                      limit_rate, cutoff_frequency);
      };
      break;
    default:
      ROS_WARN("FrankaHW: No valid control mode selected; cannot switch controllers.");
      return false;
  }
  return true;
}

void FrankaHW::init() {
  setupJointStateInterface(robot_state_);
  setupJointCommandInterface(position_joint_command_.q, robot_state_, true,
                             position_joint_interface_);
  setupJointCommandInterface(velocity_joint_command_.dq, robot_state_, true,
                             velocity_joint_interface_);
  setupJointCommandInterface(effort_joint_command_.tau_J, robot_state_, false,
                             effort_joint_interface_);
  setupLimitInterface<PositionJointSoftLimitsHandle>(urdf_model_, position_joint_limit_interface_,
                                                     position_joint_interface_);
  setupLimitInterface<VelocityJointSoftLimitsHandle>(urdf_model_, velocity_joint_limit_interface_,
                                                     velocity_joint_interface_);
  setupLimitInterface<EffortJointSoftLimitsHandle>(urdf_model_, effort_joint_limit_interface_,
                                                   effort_joint_interface_);
  setupFrankaStateInterface(robot_state_);
  setupFrankaCartesianPoseInterface(pose_cartesian_command_);
  setupFrankaCartesianVelocityInterface(velocity_cartesian_command_);
}

}  // namespace franka_hw
