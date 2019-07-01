// Copyright (c) 2019 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

#include <franka/control_types.h>
#include <franka/exception.h>
#include <franka/rate_limiting.h>
#include <franka_combinable_hw/franka_combinable_hw.h>
#include <franka_control/ErrorRecoveryAction.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <franka_hw/resource_helpers.h>

#include <pluginlib/class_list_macros.h>

#include <cstdint>
#include <exception>

#include <hardware_interface/hardware_interface.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <std_msgs/Bool.h>

using franka_control::ErrorRecoveryResult;
using franka_hw::ArmClaimedMap;
using franka_hw::getResourceMap;
using franka_hw::ResourceWithClaimsMap;

namespace franka_combinable_hw {

FrankaCombinableHW::FrankaCombinableHW()
    : effort_joint_command_libfranka_({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}),
      pose_cartesian_command_libfranka_(
          {1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0}),
      velocity_cartesian_command_libfranka_({0.0, 0.0, 0.0, 0.0, 0.0, 0.0}),
      effort_joint_command_ros_({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}),
      pose_cartesian_command_ros_(
          {1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0}),
      velocity_cartesian_command_ros_({0.0, 0.0, 0.0, 0.0, 0.0, 0.0}),
      has_error_(false),
      error_recovered_(false),
      read_write_sleep_time_(400) {}

/*
 * Data from ROS parameter server:
 *  1. const std::array<std::string, 7>& joint_names
 *  2. const std::string& arm_id
 *
    : joint_names_(joint_names),
      arm_id_(arm_id),
*/
bool FrankaCombinableHW::initROSInterfaces(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) {
  if (initialized_) {
    ROS_ERROR("FrankaCombinableHW: Cannot be initialized twice.");
    return false;
  }

  // initialize arm_id and joint_names
  if (!robot_hw_nh.getParam("arm_id", arm_id_)) {
    ROS_ERROR("FrankaCombinableHW: no parameter arm_id is found.");
    return false;
  }

  double read_write_sleep_time(0.0004);
  if (robot_hw_nh.getParam("read_write_sleep_time", read_write_sleep_time)) {
    if (read_write_sleep_time < 0.0007 && read_write_sleep_time > 0.0) {
      read_write_sleep_time_ =
          std::chrono::microseconds(static_cast<size_t>(read_write_sleep_time * 1e6));
    } else {
      ROS_WARN(
          "FrankaCombinableHW: Got invalid read_write_sleep_time. Must be in [0.0; 0.0007] "
          "seconds. Defaulting to %lu.",
          read_write_sleep_time_.count());
    }
  }

  std::vector<std::string> joint_names;
  if (robot_hw_nh.getParam("joint_names", joint_names) &&
      joint_names.size() == joint_names_.size()) {
    std::copy_n(joint_names.begin(), joint_names.size(), joint_names_.begin());
  } else {
    ROS_ERROR("FrankaCombinableHW: no valid parameter joint_names is found.");
    return false;
  }

  if (!robot_hw_nh.getParam("joint_limit_warning_threshold", arm_id_)) {
    ROS_INFO(
        "FrankaCombinableHW: no parameter joint_limit_warning_threshold is found, using default "
        "value %f",
        joint_limit_warning_threshold_);
  }

  for (size_t i = 0; i < joint_names_.size(); ++i) {
    hardware_interface::JointStateHandle joint_handle_q(joint_names_[i], &robot_state_ros_.q[i],
                                                        &robot_state_ros_.dq[i],
                                                        &robot_state_ros_.tau_J[i]);

    joint_state_interface_.registerHandle(joint_handle_q);

    hardware_interface::JointHandle effort_joint_handle(joint_handle_q,
                                                        &effort_joint_command_ros_.tau_J[i]);
    effort_joint_interface_.registerHandle(effort_joint_handle);
  }

  if (root_nh.hasParam("robot_description")) {
    if (!urdf_model_.initParamWithNodeHandle("robot_description", root_nh)) {
      ROS_ERROR("FrankaCombinableHW: Could not initialize urdf model from robot_description");
    } else {
      joint_limits_interface::SoftJointLimits soft_limits;
      joint_limits_interface::JointLimits joint_limits;

      for (const auto& joint_name : joint_names_) {
        int joint_index(std::stoi(joint_name.substr(joint_name.size() - 1)) - 1);
        auto urdf_joint = urdf_model_.getJoint(joint_name);
        if (!urdf_joint) {
          ROS_ERROR_STREAM("FrankaCombinableHW: Could not get joint " << joint_name
                                                                      << " from urdf");
        }
        if (!urdf_joint->safety) {
          ROS_ERROR_STREAM("FrankaCombinableHW: Joint " << joint_name << " has no safety");
        }
        if (!urdf_joint->limits) {
          ROS_ERROR_STREAM("FrankaCombinableHW: Joint " << joint_name << " has no limits");
        }

        if (joint_limits_interface::getSoftJointLimits(urdf_joint, soft_limits)) {
          if (joint_limits_interface::getJointLimits(urdf_joint, joint_limits)) {
            joint_limits.max_acceleration = franka::kMaxJointAcceleration[joint_index];
            joint_limits.has_acceleration_limits = true;
            joint_limits.max_jerk = franka::kMaxJointJerk[joint_index];
            joint_limits.has_jerk_limits = true;
            joint_limits_interface::EffortJointSoftLimitsHandle effort_limit_handle(
                effort_joint_interface_.getHandle(joint_name), joint_limits, soft_limits);
            effort_joint_limit_interface_.registerHandle(effort_limit_handle);
          } else {
            ROS_ERROR_STREAM("FrankaCombinableHW: Could not parse joint limit for joint "
                             << joint_name << " for joint limit interfaces");
          }
        } else {
          ROS_ERROR_STREAM("FrankaCombinableHW: Could not parse soft joint limit for joint "
                           << joint_name << " for joint limit interfaces");
        }
      }
    }
  } else {
    ROS_WARN("FrankaCombinableHW: No parameter robot_description found to set joint limits!");
  }

  franka_hw::FrankaStateHandle franka_state_handle(arm_id_ + "_robot", robot_state_ros_);
  franka_state_interface_.registerHandle(franka_state_handle);
  franka_hw::FrankaCartesianPoseHandle franka_cartesian_pose_handle(
      franka_state_handle, pose_cartesian_command_ros_.O_T_EE, pose_cartesian_command_ros_.elbow);
  franka_pose_cartesian_interface_.registerHandle(franka_cartesian_pose_handle);
  franka_hw::FrankaCartesianVelocityHandle franka_cartesian_velocity_handle(
      franka_state_handle, velocity_cartesian_command_ros_.O_dP_EE,
      velocity_cartesian_command_ros_.elbow);
  franka_velocity_cartesian_interface_.registerHandle(franka_cartesian_velocity_handle);

  registerInterface(&franka_state_interface_);
  registerInterface(&joint_state_interface_);
  registerInterface(&effort_joint_interface_);
  registerInterface(&franka_pose_cartesian_interface_);
  registerInterface(&franka_velocity_cartesian_interface_);

  // Setup error state publisher
  has_error_pub_ = robot_hw_nh.advertise<std_msgs::Bool>("has_error", 1, true);
  publishErrorState(has_error_);

  // Setup ROS services and action server
  setupServicesAndActionServers(robot_hw_nh);

  initialized_ = true;
  return true;
}

/*
 * Data from ROS parameter server:
 *  1. const std::array<std::string, 7>& joint_names
 *  2. const std::string& arm_id
 *
    : joint_names_(joint_names),
      arm_id_(arm_id),
*/
bool FrankaCombinableHW::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) {
  // Initialize ROS-interfaces
  if (not initROSInterfaces(root_nh, robot_hw_nh)) {
    ROS_ERROR("FrankaCombinableHW: Failed to initialize interfaces.");
    return false;
  }

  // Initialize and connect to franka::Robot, register model interface and start control loop
  try {
    if (!initRobot(robot_hw_nh)) {
      ROS_ERROR("FrankaCombinableHW: Failed to initialize libfranka robot.");
      return false;
    }
  } catch (const std::runtime_error& error) {
    ROS_ERROR("FrankaCombinableHW: Failed to initialize libfranka robot. %s", error.what());
    return false;
  }

  return true;
}

bool FrankaCombinableHW::initRobot(ros::NodeHandle& robot_hw_nh) {
  if (!robot_hw_nh.getParam("robot_ip", robot_ip_)) {
    ROS_ERROR("FrankaCombinableHW: no parameter robot_ip is found.");
    return false;
  }

  robot_ = std::make_unique<franka::Robot>(robot_ip_);

  // Set default collision behaviour
  robot_->setCollisionBehavior(
      {{40.0, 40.0, 38.0, 38.0, 32.0, 30.0, 24.0}}, {{40.0, 40.0, 38.0, 36.0, 32.0, 30.0, 24.0}},
      {{40.0, 40.0, 38.0, 38.0, 32.0, 30.0, 24.0}}, {{40.0, 40.0, 38.0, 36.0, 32.0, 30.0, 24.0}},
      {{40.0, 40.0, 40.0, 50.0, 50.0, 50.0}}, {{40.0, 40.0, 40.0, 50.0, 50.0, 50.0}},
      {{40.0, 40.0, 40.0, 50.0, 50.0, 50.0}}, {{40.0, 40.0, 40.0, 50.0, 50.0, 50.0}});

  // Initialize robot state before loading any controller
  update(robot_->readOnce());

  model_ = std::make_unique<franka::Model>(robot_->loadModel());
  franka_hw::FrankaModelHandle model_handle(arm_id_ + "_model", *model_, robot_state_ros_);
  franka_model_interface_.registerHandle(model_handle);
  registerInterface(&franka_model_interface_);
  control_loop_thread_ = std::make_unique<std::thread>(&FrankaCombinableHW::controlLoop, this);
  return true;
}

void FrankaCombinableHW::publishErrorState(bool error) {
  std_msgs::Bool msg;
  msg.data = error;  // NOLINT (readability-implicit-bool-cast)
  has_error_pub_.publish(msg);
}

void FrankaCombinableHW::checkJointLimits() {
  // Check joint limits and print warning if any joint is close to limit
  std::string joint_limits_warning;
  for (const auto& k_joint_name : joint_names_) {
    try {
      auto joint_handle = joint_state_interface_.getHandle(k_joint_name);
      auto urdf_joint = urdf_model_.getJoint(k_joint_name);
      joint_limits_interface::JointLimits joint_limits;
      if (joint_limits_interface::getJointLimits(urdf_joint, joint_limits)) {
        double joint_lower = joint_limits.min_position;
        double joint_upper = joint_limits.max_position;
        double joint_position = joint_handle.getPosition();
        double dist = fmin(fabs(joint_position - joint_lower), fabs(joint_position - joint_upper));
        if (dist < joint_limit_warning_threshold_) {
          joint_limits_warning += "\n\t" + k_joint_name + ": " + std::to_string(dist * 180 / 3.14) +
                                  " degrees to joint limits (limits: [" +
                                  std::to_string(joint_lower) + ", " + std::to_string(joint_upper) +
                                  "]" + " q: " + std::to_string(joint_position) + ") ";
        }
      } else {
        ROS_ERROR_STREAM_ONCE("FrankaCombinableHW: Could not parse joint limit for joint "
                              << k_joint_name << " for joint limit interfaces");
      }
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM_ONCE("FrankaCombinableHW: Could not get joint handle " << k_joint_name
                                                                              << " .\n"
                                                                              << ex.what());
      return;
    }
  }

  if (!joint_limits_warning.empty()) {
    ROS_WARN_THROTTLE(5, "FrankaCombinableHW: %s", joint_limits_warning.c_str());
  }
}

void FrankaCombinableHW::update(const franka::RobotState& robot_state) {
  ros_state_mutex_.lock();
  robot_state_ros_ = robot_state;
  ros_state_mutex_.unlock();
}

bool FrankaCombinableHW::controllerActive() const noexcept {
  return controller_active_;
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
    pose_cartesian_command_libfranka_ = franka::CartesianPose(robot_state_libfranka_.O_T_EE_d);
    effort_joint_command_libfranka_ = franka::Torques({0., 0., 0., 0., 0., 0., 0.});
    velocity_cartesian_command_libfranka_ = franka::CartesianVelocities({0., 0., 0., 0., 0., 0.});
    libfranka_cmd_mutex_.unlock();

    try {
      // Run control loop. It will exit if the controller is switched.
      control(*robot_);
    } catch (const franka::ControlException& e) {
      // Reflex could be caught and it needs to wait for automatic error recovery
      ROS_ERROR("%s", e.what());
      has_error_ = true;
      publishErrorState(has_error_);
    }
  }
}

void FrankaCombinableHW::setupServicesAndActionServers(ros::NodeHandle& node_handle) {
  services_
      .advertiseService<franka_control::SetJointImpedance>(
          node_handle, "set_joint_impedance",
          [this](auto&& req, auto&& res) {
            return franka_control::setJointImpedance(*(this->robot_), req, res);
          })
      .advertiseService<franka_control::SetCartesianImpedance>(
          node_handle, "set_cartesian_impedance",
          [this](auto&& req, auto&& res) {
            return franka_control::setCartesianImpedance(*(this->robot_), req, res);
          })
      .advertiseService<franka_control::SetEEFrame>(node_handle, "set_EE_frame",
                                                    [this](auto&& req, auto&& res) {
                                                      return franka_control::setEEFrame(
                                                          *(this->robot_), req, res);
                                                    })
      .advertiseService<franka_control::SetKFrame>(node_handle, "set_K_frame",
                                                   [this](auto&& req, auto&& res) {
                                                     return franka_control::setKFrame(
                                                         *(this->robot_), req, res);
                                                   })
      .advertiseService<franka_control::SetForceTorqueCollisionBehavior>(
          node_handle, "set_force_torque_collision_behavior",
          [this](auto&& req, auto&& res) {
            return franka_control::setForceTorqueCollisionBehavior(*(this->robot_), req, res);
          })
      .advertiseService<franka_control::SetFullCollisionBehavior>(
          node_handle, "set_full_collision_behavior",
          [this](auto&& req, auto&& res) {
            return franka_control::setFullCollisionBehavior(*(this->robot_), req, res);
          })
      .advertiseService<franka_control::SetLoad>(
          node_handle, "set_load", [this](auto&& req, auto&& res) {
            return franka_control::setLoad(*(this->robot_), req, res);
          });

  recovery_action_server_ =
      std::make_unique<actionlib::SimpleActionServer<franka_control::ErrorRecoveryAction>>(
          node_handle, "error_recovery",
          [&](const franka_control::ErrorRecoveryGoalConstPtr&) {
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
              recovery_action_server_->setAborted(ErrorRecoveryResult(), ex.what());
            }
          },
          false);
  recovery_action_server_->start();
}

void FrankaCombinableHW::control(franka::Robot& robot) {
  if (!controller_active_) {
    return;
  }
  run_function_(robot);
}

void FrankaCombinableHW::enforceLimits(const ros::Duration& period) {
  if (period.toSec() > 0.0) {
    effort_joint_limit_interface_.enforceLimits(period);
  }
}

bool FrankaCombinableHW::checkForConflict(
    const std::list<hardware_interface::ControllerInfo>& info) const {
  ResourceWithClaimsMap resource_map = getResourceMap(info);
  // check for conflicts in single resources: no triple claims,
  // for 2 claims it must be one torque and one non-torque claim
  for (auto map_it = resource_map.begin(); map_it != resource_map.end(); map_it++) {
    if (map_it->second.size() > 2) {
      ROS_ERROR_STREAM("FrankaCombinableHW: Resource "
                       << map_it->first << " claimed with more than two interfaces. Conflict!");
      return true;
    }
    uint8_t torque_claims = 0;
    uint8_t other_claims = 0;
    if (map_it->second.size() == 2) {
      for (auto& claimed_by : map_it->second) {
        if (claimed_by[2] == "hardware_interface::EffortJointInterface") {
          torque_claims++;
        } else {
          other_claims++;
        }
      }
      if (torque_claims == 0) {
        ROS_ERROR_STREAM("FrankaCombinableHW: Resource "
                         << map_it->first
                         << " is claimed with two non-compatible interfaces. Conflict!");
        return true;
      }
    }
  }

  ArmClaimedMap arm_claim_map;
  if (!getArmClaimedMap(resource_map, arm_claim_map)) {
    ROS_ERROR("FrankaCombinableHW: Unknown interface claimed. Conflict!");
    return true;
  }

  // check for any claim to joint_position or joint velocity interface which are not supported
  if (arm_claim_map.find(arm_id_) != arm_claim_map.end()) {
    if (arm_claim_map[arm_id_].joint_position_claims +
            arm_claim_map[arm_id_].joint_velocity_claims >
        0) {
      ROS_ERROR_STREAM("FrankaCombinableHW: Invalid claim joint position or velocity interface."
                       << "Note: joint position and joint velocity interfaces are not supported"
                       << " in FrankaCombinableHW. Arm:" << arm_id_ << ". Conflict!");
      return true;
    }
  }

  // check for any claim to cartesian interfaces without torque interface
  if (arm_claim_map.find(arm_id_) != arm_claim_map.end()) {
    if (arm_claim_map[arm_id_].cartesian_velocity_claims +
                arm_claim_map[arm_id_].cartesian_pose_claims >
            0 &&
        arm_claim_map[arm_id_].joint_torque_claims == 0) {
      ROS_ERROR_STREAM(
          "FrankaCombinableHW: Invalid claim cartesian interface without joint torque"
          " interface. Arm:"
          << arm_id_ << ". Conflict!");
      return true;
    }
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
      ROS_ERROR_STREAM("FrankaCombinableHW: Invalid claims on joint AND cartesian level on arm "
                       << arm_id_ << ". Conflict!");
      return true;
    }
    if ((arm_claim_map[arm_id_].joint_position_claims > 0 &&
         arm_claim_map[arm_id_].joint_position_claims != 7) ||
        (arm_claim_map[arm_id_].joint_velocity_claims > 0 &&
         arm_claim_map[arm_id_].joint_velocity_claims != 7) ||
        (arm_claim_map[arm_id_].joint_torque_claims > 0 &&
         arm_claim_map[arm_id_].joint_torque_claims != 7)) {
      ROS_ERROR_STREAM("FrankaCombinableHW: Non-consistent claims on the joints of "
                       << arm_id_ << ". Not supported. Conflict!");
      return true;
    }
  }
  return false;
}

// doSwitch runs on the main realtime thread
void FrankaCombinableHW::doSwitch(
    const std::list<hardware_interface::ControllerInfo>& /* start_list */,
    const std::list<hardware_interface::ControllerInfo>& /* stop_list */) {
  if (current_control_mode_ != ControlMode::None) {
    controller_active_ = true;
  }
}

// prepareSwitch runs on the background message handling thread
bool FrankaCombinableHW::prepareSwitch(
    const std::list<hardware_interface::ControllerInfo>& start_list,
    const std::list<hardware_interface::ControllerInfo>& stop_list) {
  ResourceWithClaimsMap start_resource_map = getResourceMap(start_list);
  ArmClaimedMap start_arm_claim_map;
  if (!getArmClaimedMap(start_resource_map, start_arm_claim_map)) {
    ROS_ERROR("FrankaCombinableHW: Unknown interface claimed for starting!");
    return false;
  }
  ControlMode start_control_mode = getControlMode(arm_id_, start_arm_claim_map);

  ResourceWithClaimsMap stop_resource_map = getResourceMap(stop_list);
  ArmClaimedMap stop_arm_claim_map;
  if (!getArmClaimedMap(stop_resource_map, stop_arm_claim_map)) {
    ROS_ERROR("FrankaCombinableHW: Unknown interface claimed for stopping!");
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
      run_function_ = [this](franka::Robot& robot) {
        robot.control(std::bind(&FrankaCombinableHW::controlCallback<franka::Torques>, this,
                                std::cref(effort_joint_command_libfranka_), _1, _2),
                      limit_rate_);
      };
      break;
    case ControlMode::JointPosition:
      ROS_WARN("FrankaCombinableHW: No valid control mode selected; cannot switch controllers.");
      return false;
    case ControlMode::JointVelocity:
      ROS_WARN("FrankaCombinableHW: No valid control mode selected; cannot switch controllers.");
      return false;
    case ControlMode::CartesianPose:
      ROS_WARN("FrankaCombinableHW: No valid control mode selected; cannot switch controllers.");
      return false;
    case ControlMode::CartesianVelocity:
      ROS_WARN("FrankaCombinableHW: No valid control mode selected; cannot switch controllers.");
      return false;
    case (ControlMode::JointTorque | ControlMode::JointPosition):
      ROS_WARN("FrankaCombinableHW: No valid control mode selected; cannot switch controllers.");
      return false;
    case (ControlMode::JointTorque | ControlMode::JointVelocity):
      ROS_WARN("FrankaCombinableHW: No valid control mode selected; cannot switch controllers.");
      return false;
    case (ControlMode::JointTorque | ControlMode::CartesianPose):
      run_function_ = [this](franka::Robot& robot) {
        robot.control(std::bind(&FrankaCombinableHW::controlCallback<franka::Torques>, this,
                                std::cref(effort_joint_command_libfranka_), _1, _2),
                      std::bind(&FrankaCombinableHW::controlCallback<franka::CartesianPose>, this,
                                std::cref(pose_cartesian_command_libfranka_), _1, _2),
                      limit_rate_);
      };
      break;
    case (ControlMode::JointTorque | ControlMode::CartesianVelocity):
      run_function_ = [this](franka::Robot& robot) {
        robot.control(std::bind(&FrankaCombinableHW::controlCallback<franka::Torques>, this,
                                std::cref(effort_joint_command_libfranka_), _1, _2),
                      std::bind(&FrankaCombinableHW::controlCallback<franka::CartesianVelocities>,
                                this, std::cref(velocity_cartesian_command_libfranka_), _1, _2),
                      limit_rate_);
      };
      break;
    default:
      ROS_WARN("FrankaCombinableHW: No valid control mode selected; cannot switch controllers.");
      return false;
  }

  if (current_control_mode_ != requested_control_mode) {
    ROS_INFO_STREAM("FrankaCombinableHW: Prepared switching controllers to "
                    << requested_control_mode);
    current_control_mode_ = requested_control_mode;

    controller_active_ = false;
  }

  return true;
}

void FrankaCombinableHW::read(const ros::Time&,        // NOLINT (readability-identifier-naming)
                              const ros::Duration&) {  // NOLINT [readability-named-parameter]
  controller_needs_reset_ = error_recovered_;
  ros_state_mutex_.lock();
  libfranka_state_mutex_.lock();
  robot_state_ros_ = robot_state_libfranka_;
  libfranka_state_mutex_.unlock();
  ros_state_mutex_.unlock();
}

void FrankaCombinableHW::write(const ros::Time&,  // NOLINT (readability-identifier-naming)
                               const ros::Duration& period) {
  // if flag `controller_needs_reset_` was updated, then controller_manager. update(...,
  // reset_controller) must
  // have been executed to reset the controller.
  if (controller_needs_reset_ && error_recovered_) {
    controller_needs_reset_ = false;
    error_recovered_ = false;
  }
  enforceLimits(period);
  ros_cmd_mutex_.lock();
  libfranka_cmd_mutex_.lock();
  effort_joint_command_libfranka_.tau_J =
      saturateTorqueRate(effort_joint_command_ros_.tau_J, robot_state_libfranka_.tau_J_d);
  pose_cartesian_command_libfranka_ = pose_cartesian_command_ros_;
  velocity_cartesian_command_libfranka_ = velocity_cartesian_command_ros_;
  libfranka_cmd_mutex_.unlock();
  ros_cmd_mutex_.unlock();
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

std::array<double, 7> FrankaCombinableHW::getJointEffortCommand() const noexcept {
  return effort_joint_command_libfranka_.tau_J;
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

bool FrankaCombinableHW::commandHasNaN(const franka::Torques& command) {
  return arrayHasNaN(command.tau_J);
}

bool FrankaCombinableHW::commandHasNaN(const franka::CartesianPose& command) {
  return arrayHasNaN(command.elbow) || arrayHasNaN(command.O_T_EE);
}

bool FrankaCombinableHW::commandHasNaN(const franka::CartesianVelocities& command) {
  return arrayHasNaN(command.elbow) || arrayHasNaN(command.O_dP_EE);
}

}  // namespace franka_combinable_hw

PLUGINLIB_EXPORT_CLASS(franka_combinable_hw::FrankaCombinableHW, hardware_interface::RobotHW)
