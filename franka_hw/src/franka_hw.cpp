// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_hw/franka_hw.h>

#include <cstdint>

#include <joint_limits_interface/joint_limits_urdf.h>
#include <urdf/model.h>

#include "resource_helpers.h"

namespace franka_hw {

constexpr double FrankaHW::kMaximumJointAcceleration;
constexpr double FrankaHW::kMaximumJointJerk;

FrankaHW::FrankaHW(const std::array<std::string, 7>& joint_names,
                   const std::string& arm_id,
                   const ros::NodeHandle& node_handle)
    : joint_names_(joint_names),
      arm_id_(arm_id),
      position_joint_command_({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}),
      velocity_joint_command_({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}),
      effort_joint_command_({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}),
      pose_cartesian_command_(
          {1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0}),
      velocity_cartesian_command_({0.0, 0.0, 0.0, 0.0, 0.0, 0.0}) {
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    hardware_interface::JointStateHandle joint_handle_q_d(
        joint_names_[i], &robot_state_.q_d[i], &robot_state_.dq_d[i], &robot_state_.tau_J[i]);

    hardware_interface::JointStateHandle joint_handle_q(
        joint_names_[i], &robot_state_.q[i], &robot_state_.dq[i], &robot_state_.tau_J[i]);

    joint_state_interface_.registerHandle(joint_handle_q);

    hardware_interface::JointHandle position_joint_handle(joint_handle_q_d,
                                                          &position_joint_command_.q[i]);
    position_joint_interface_.registerHandle(position_joint_handle);

    hardware_interface::JointHandle velocity_joint_handle(joint_handle_q_d,
                                                          &velocity_joint_command_.dq[i]);
    velocity_joint_interface_.registerHandle(velocity_joint_handle);

    hardware_interface::JointHandle effort_joint_handle(joint_handle_q,
                                                        &effort_joint_command_.tau_J[i]);
    effort_joint_interface_.registerHandle(effort_joint_handle);
  }

  if (node_handle.hasParam("robot_description")) {
    urdf::Model urdf_model;
    if (!urdf_model.initParamWithNodeHandle("robot_description", node_handle)) {
      ROS_ERROR("FrankaHW: Could not initialize urdf model from robot_description");
    } else {
      joint_limits_interface::SoftJointLimits soft_limits;
      joint_limits_interface::JointLimits joint_limits;

      for (auto joint_name : joint_names_) {
        auto urdf_joint = urdf_model.getJoint(joint_name);
        if (!urdf_joint) {
          ROS_ERROR_STREAM("FrankaHW: Could not get joint " << joint_name << " from urdf");
        }
        if (!urdf_joint->safety) {
          ROS_ERROR_STREAM("FrankaHW: Joint " << joint_name << " has no safety");
        }
        if (!urdf_joint->limits) {
          ROS_ERROR_STREAM("FrankaHW: Joint " << joint_name << " has no limits");
        }

        if (joint_limits_interface::getSoftJointLimits(urdf_joint, soft_limits)) {
          if (joint_limits_interface::getJointLimits(urdf_joint, joint_limits)) {
            joint_limits.max_acceleration = kMaximumJointAcceleration;
            joint_limits.has_acceleration_limits = true;
            joint_limits.max_jerk = kMaximumJointJerk;
            joint_limits.has_jerk_limits = true;
            joint_limits_interface::PositionJointSoftLimitsHandle position_limit_handle(
                position_joint_interface_.getHandle(joint_name), joint_limits, soft_limits);
            position_joint_limit_interface_.registerHandle(position_limit_handle);

            joint_limits_interface::VelocityJointSoftLimitsHandle velocity_limit_handle(
                velocity_joint_interface_.getHandle(joint_name), joint_limits, soft_limits);
            velocity_joint_limit_interface_.registerHandle(velocity_limit_handle);

            joint_limits_interface::EffortJointSoftLimitsHandle effort_limit_handle(
                effort_joint_interface_.getHandle(joint_name), joint_limits, soft_limits);
            effort_joint_limit_interface_.registerHandle(effort_limit_handle);
          } else {
            ROS_ERROR_STREAM("FrankaHW: Could not parse joint limit for joint "
                             << joint_name << " for joint limit interfaces");
          }
        } else {
          ROS_ERROR_STREAM("FrankaHW: Could not parse soft joint limit for joint "
                           << joint_name << " for joint limit interfaces");
        }
      }
    }
  } else {
    ROS_WARN("FrankaHW: No parameter robot_description found to set joint limits!");
  }

  FrankaStateHandle franka_state_handle(arm_id_ + "_robot", robot_state_);
  franka_state_interface_.registerHandle(franka_state_handle);
  FrankaCartesianPoseHandle franka_cartesian_pose_handle(
      franka_state_handle, pose_cartesian_command_.O_T_EE, pose_cartesian_command_.elbow);
  franka_pose_cartesian_interface_.registerHandle(franka_cartesian_pose_handle);
  FrankaCartesianVelocityHandle franka_cartesian_velocity_handle(
      franka_state_handle, velocity_cartesian_command_.O_dP_EE, velocity_cartesian_command_.elbow);
  franka_velocity_cartesian_interface_.registerHandle(franka_cartesian_velocity_handle);

  registerInterface(&franka_state_interface_);
  registerInterface(&joint_state_interface_);
  registerInterface(&position_joint_interface_);
  registerInterface(&velocity_joint_interface_);
  registerInterface(&effort_joint_interface_);
  registerInterface(&franka_pose_cartesian_interface_);
  registerInterface(&franka_velocity_cartesian_interface_);
}

FrankaHW::FrankaHW(const std::array<std::string, 7>& joint_names,
                   const std::string& arm_id,
                   const ros::NodeHandle& node_handle,
                   franka::Model& model)
    : FrankaHW(joint_names, arm_id, node_handle) {
  franka_hw::FrankaModelHandle model_handle(arm_id_ + "_model", model, robot_state_);

  franka_model_interface_.registerHandle(model_handle);

  registerInterface(&franka_model_interface_);
}

void FrankaHW::update(const franka::RobotState& robot_state) {
  robot_state_ = robot_state;
}

bool FrankaHW::controllerActive() const noexcept {
  return controller_active_;
}

void FrankaHW::control(franka::Robot& robot,
                       std::function<bool(const ros::Time&, const ros::Duration&)> ros_callback) {
  if (!controller_active_) {
    return;
  }

  franka::Duration last_time = robot_state_.time;

  run_function_(robot, [this, ros_callback, &last_time](const franka::RobotState& robot_state,
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

bool FrankaHW::checkForConflict(const std::list<hardware_interface::ControllerInfo>& info) const {
  ResourceWithClaimsMap resource_map = getResourceMap(info);
  // check for conflicts in single resources: no triple claims,
  // for 2 claims it must be one torque and one non-torque claim
  for (auto map_it = resource_map.begin(); map_it != resource_map.end(); map_it++) {
    if (map_it->second.size() > 2) {
      ROS_ERROR_STREAM("FrankaHW: Resource "
                       << map_it->first << " claimed with more than two interfaces. Conflict!");
      return true;
    }
    uint8_t torque_claims = 0;
    uint8_t other_claims = 0;
    if (map_it->second.size() == 2) {
      for (auto& claimed_by : map_it->second) {
        if (claimed_by[2].compare("hardware_interface::EffortJointInterface") == 0) {
          torque_claims++;
        } else {
          other_claims++;
        }
      }
      if (torque_claims != 1) {
        ROS_ERROR_STREAM("FrankaHW: Resource "
                         << map_it->first
                         << " is claimed with two non-compatible interfaces. Conflict!");
        return true;
      }
    }
  }

  ArmClaimedMap arm_claim_map;
  if (!getArmClaimedMap(resource_map, arm_claim_map)) {
    ROS_ERROR_STREAM("FrankaHW: Unknown interface claimed. Conflict!");
    return true;
  }

  // check for conflicts between joint and cartesian level for each arm.
  // Valid claims are torque claims on joint level in combination with either
  // 7 non-torque claims on joint_level or one claim on cartesian level.
  if (arm_claim_map.find(arm_id_) != arm_claim_map.end()) {
    if ((arm_claim_map[arm_id_].cartesian_velocity_claims +
                 arm_claim_map[arm_id_].cartesian_pose_claims >
             0 &&
         arm_claim_map[arm_id_].joint_position_claims +
                 arm_claim_map[arm_id_].joint_velocity_claims >
             0)) {
      ROS_ERROR_STREAM("FrankaHW: Invalid claims on joint AND cartesian level on arm "
                       << arm_id_ << ". Conflict!");
      return true;
    }
    if ((arm_claim_map[arm_id_].joint_position_claims > 0 &&
         arm_claim_map[arm_id_].joint_position_claims != 7) ||
        (arm_claim_map[arm_id_].joint_velocity_claims > 0 &&
         arm_claim_map[arm_id_].joint_velocity_claims != 7) ||
        (arm_claim_map[arm_id_].joint_torque_claims > 0 &&
         arm_claim_map[arm_id_].joint_torque_claims != 7)) {
      ROS_ERROR_STREAM("FrankaHW: Non-consistent claims on the joints of "
                       << arm_id_ << ". Not supported. Conflict!");
      return true;
    }
  }
  return false;
}

// doSwitch runs on the main realtime thread
void FrankaHW::doSwitch(const std::list<hardware_interface::ControllerInfo>& /* start_list */,
                        const std::list<hardware_interface::ControllerInfo>& /* stop_list */) {
  if (current_control_mode_ != ControlMode::None) {
    reset();
    controller_active_ = true;
  }
}

// prepareSwitch runs on the background message handling thread
bool FrankaHW::prepareSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                             const std::list<hardware_interface::ControllerInfo>& stop_list) {
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
  requested_control_mode |= start_control_mode;
  requested_control_mode &= ~stop_control_mode;

  using std::placeholders::_1;
  using std::placeholders::_2;

  switch (requested_control_mode) {
    case ControlMode::None:
      break;
    case ControlMode::JointTorque:
      run_function_ = [this](franka::Robot& robot, Callback ros_callback) {
        robot.control(std::bind(&FrankaHW::controlCallback<franka::Torques>, this,
                                std::cref(effort_joint_command_), ros_callback, _1, _2));
      };
      break;
    case ControlMode::JointPosition:
      run_function_ = [this](franka::Robot& robot, Callback ros_callback) {
        robot.control(std::bind(&FrankaHW::controlCallback<franka::JointPositions>, this,
                                std::cref(position_joint_command_), ros_callback, _1, _2));
      };
      break;
    case ControlMode::JointVelocity:
      run_function_ = [this](franka::Robot& robot, Callback ros_callback) {
        robot.control(std::bind(&FrankaHW::controlCallback<franka::JointVelocities>, this,
                                std::cref(velocity_joint_command_), ros_callback, _1, _2));
      };
      break;
    case ControlMode::CartesianPose:
      run_function_ = [this](franka::Robot& robot, Callback ros_callback) {
        robot.control(std::bind(&FrankaHW::controlCallback<franka::CartesianPose>, this,
                                std::cref(pose_cartesian_command_), ros_callback, _1, _2));
      };
      break;
    case ControlMode::CartesianVelocity:
      run_function_ = [this](franka::Robot& robot, Callback ros_callback) {
        robot.control(std::bind(&FrankaHW::controlCallback<franka::CartesianVelocities>, this,
                                std::cref(velocity_cartesian_command_), ros_callback, _1, _2));
      };
      break;
    case (ControlMode::JointTorque | ControlMode::JointPosition):
      run_function_ = [this](franka::Robot& robot, Callback ros_callback) {
        robot.control(std::bind(&FrankaHW::controlCallback<franka::Torques>, this,
                                std::cref(effort_joint_command_), ros_callback, _1, _2),
                      std::bind(&FrankaHW::controlCallback<franka::JointPositions>, this,
                                std::cref(position_joint_command_), ros_callback, _1, _2));
      };
      break;
    case (ControlMode::JointTorque | ControlMode::JointVelocity):
      run_function_ = [this](franka::Robot& robot, Callback ros_callback) {
        robot.control(std::bind(&FrankaHW::controlCallback<franka::Torques>, this,
                                std::cref(effort_joint_command_), ros_callback, _1, _2),
                      std::bind(&FrankaHW::controlCallback<franka::JointVelocities>, this,
                                std::cref(velocity_joint_command_), ros_callback, _1, _2));
      };
      break;
    case (ControlMode::JointTorque | ControlMode::CartesianPose):
      run_function_ = [this](franka::Robot& robot, Callback ros_callback) {
        robot.control(std::bind(&FrankaHW::controlCallback<franka::Torques>, this,
                                std::cref(effort_joint_command_), ros_callback, _1, _2),
                      std::bind(&FrankaHW::controlCallback<franka::CartesianPose>, this,
                                std::cref(pose_cartesian_command_), ros_callback, _1, _2));
      };
      break;
    case (ControlMode::JointTorque | ControlMode::CartesianVelocity):
      run_function_ = [this](franka::Robot& robot, Callback ros_callback) {
        robot.control(std::bind(&FrankaHW::controlCallback<franka::Torques>, this,
                                std::cref(effort_joint_command_), ros_callback, _1, _2),
                      std::bind(&FrankaHW::controlCallback<franka::CartesianVelocities>, this,
                                std::cref(velocity_cartesian_command_), ros_callback, _1, _2));
      };
      break;
    default:
      ROS_WARN("FrankaHW: No valid control mode selected; cannot switch controllers.");
      return false;
  }

  if (current_control_mode_ != requested_control_mode) {
    ROS_INFO_STREAM("FrankaHW: Prepared switching controllers to " << requested_control_mode);
    current_control_mode_ = requested_control_mode;

    controller_active_ = false;
  }

  return true;
}

std::array<double, 7> FrankaHW::getJointPositionCommand() const noexcept {
  return position_joint_command_.q;
}

std::array<double, 7> FrankaHW::getJointVelocityCommand() const noexcept {
  return velocity_joint_command_.dq;
}

std::array<double, 7> FrankaHW::getJointEffortCommand() const noexcept {
  return effort_joint_command_.tau_J;
}

void FrankaHW::reset() {
  position_joint_limit_interface_.reset();
}

}  // namespace franka_hw
