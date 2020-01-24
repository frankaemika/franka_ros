// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_hw/franka_hw.h>
#include <franka_hw/resource_helpers.h>

#include <array>
#include <cstdint>
#include <exception>
#include <functional>
#include <list>
#include <mutex>
#include <ostream>
#include <sstream>
#include <string>
#include <utility>

#include <franka/control_types.h>
#include <franka/rate_limiting.h>
#include <franka/robot.h>
#include <joint_limits_interface/joint_limits_urdf.h>

namespace franka_hw {

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

std::string toStringWithPrecision(const double value, const size_t precision = 6) {
  std::ostringstream out;
  out.precision(precision);
  out << std::fixed << value;
  return out.str();
}

}  // anonymous namespace

FrankaHW::FrankaHW()
    : position_joint_command_ros_({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}),
      position_joint_command_libfranka_({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}),
      velocity_joint_command_ros_({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}),
      velocity_joint_command_libfranka_({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}),
      effort_joint_command_ros_({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}),
      effort_joint_command_libfranka_({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}),
      pose_cartesian_command_ros_(
          {1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0}),
      pose_cartesian_command_libfranka_(
          {1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0}),
      velocity_cartesian_command_ros_({0.0, 0.0, 0.0, 0.0, 0.0, 0.0}),
      velocity_cartesian_command_libfranka_({0.0, 0.0, 0.0, 0.0, 0.0, 0.0}) {}

bool FrankaHW::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) {
  if (initialized_) {
    ROS_ERROR("FrankaHW: Cannot be initialized twice.");
    return false;
  }

  if (!initParameters(root_nh, robot_hw_nh)) {
    ROS_ERROR("FrankaHW: Failed to parse all required parameters.");
    return false;
  }
  try {
    initRobot();
  } catch (const std::runtime_error& error) {
    ROS_ERROR("FrankaHW: Failed to initialize libfranka robot. %s", error.what());
    return false;
  }
  initROSInterfaces(robot_hw_nh);
  setupParameterCallbacks(robot_hw_nh);

  initialized_ = true;
  return true;
}

bool FrankaHW::initParameters(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) {
  std::vector<std::string> joint_names_vector;
  if (!robot_hw_nh.getParam("joint_names", joint_names_vector) || joint_names_vector.size() != 7) {
    ROS_ERROR("Invalid or no joint_names parameters provided");
    return false;
  }
  std::copy(joint_names_vector.cbegin(), joint_names_vector.cend(), joint_names_.begin());

  bool rate_limiting;
  if (!robot_hw_nh.getParamCached("rate_limiting", rate_limiting)) {
    ROS_ERROR("Invalid or no rate_limiting parameter provided");
    return false;
  }

  double cutoff_frequency;
  if (!robot_hw_nh.getParamCached("cutoff_frequency", cutoff_frequency)) {
    ROS_ERROR("Invalid or no cutoff_frequency parameter provided");
    return false;
  }

  std::string internal_controller;
  if (!robot_hw_nh.getParam("internal_controller", internal_controller)) {
    ROS_ERROR("No internal_controller parameter provided");
    return false;
  }

  if (!robot_hw_nh.getParam("arm_id", arm_id_)) {
    ROS_ERROR("Invalid or no arm_id parameter provided");
    return false;
  }

  if (!urdf_model_.initParamWithNodeHandle("robot_description", root_nh)) {
    ROS_ERROR("Could not initialize URDF model from robot_description");
    return false;
  }

  if (!robot_hw_nh.getParam("robot_ip", robot_ip_)) {
    ROS_ERROR("Invalid or no robot_ip parameter provided");
    return false;
  }

  if (!robot_hw_nh.getParam("joint_limit_warning_threshold", joint_limit_warning_threshold_)) {
    ROS_INFO(
        "No parameter joint_limit_warning_threshold is found, using default "
        "value %f",
        joint_limit_warning_threshold_);
  }

  std::string realtime_config_param = robot_hw_nh.param("realtime_config", std::string("enforce"));
  if (realtime_config_param == "enforce") {
    realtime_config_ = franka::RealtimeConfig::kEnforce;
  } else if (realtime_config_param == "ignore") {
    realtime_config_ = franka::RealtimeConfig::kIgnore;
  } else {
    ROS_ERROR("Invalid realtime_config parameter provided. Valid values are 'enforce', 'ignore'.");
    return false;
  }

  // Get full collision behavior config from the parameter server.
  std::vector<double> thresholds =
      getCollisionThresholds("lower_torque_thresholds_acceleration", robot_hw_nh,
                             {20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0});
  std::copy(thresholds.begin(), thresholds.end(),
            collision_config_.lower_torque_thresholds_acceleration.begin());
  thresholds = getCollisionThresholds("upper_torque_thresholds_acceleration", robot_hw_nh,
                                      {20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0});
  std::copy(thresholds.begin(), thresholds.end(),
            collision_config_.upper_torque_thresholds_acceleration.begin());
  thresholds = getCollisionThresholds("lower_torque_thresholds_nominal", robot_hw_nh,
                                      {20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0});
  std::copy(thresholds.begin(), thresholds.end(),
            collision_config_.lower_torque_thresholds_nominal.begin());
  thresholds = getCollisionThresholds("upper_torque_thresholds_nominal", robot_hw_nh,
                                      {20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0});
  std::copy(thresholds.begin(), thresholds.end(),
            collision_config_.upper_torque_thresholds_nominal.begin());
  thresholds.resize(6);
  thresholds = getCollisionThresholds("lower_force_thresholds_acceleration", robot_hw_nh,
                                      {20.0, 20.0, 20.0, 25.0, 25.0, 25.0});
  std::copy(thresholds.begin(), thresholds.end(),
            collision_config_.lower_force_thresholds_acceleration.begin());
  thresholds = getCollisionThresholds("upper_force_thresholds_acceleration", robot_hw_nh,
                                      {20.0, 20.0, 20.0, 25.0, 25.0, 25.0});
  std::copy(thresholds.begin(), thresholds.end(),
            collision_config_.upper_force_thresholds_acceleration.begin());
  thresholds = getCollisionThresholds("lower_force_thresholds_nominal", robot_hw_nh,
                                      {20.0, 20.0, 20.0, 25.0, 25.0, 25.0});
  std::copy(thresholds.begin(), thresholds.end(),
            collision_config_.lower_force_thresholds_nominal.begin());
  thresholds = getCollisionThresholds("upper_force_thresholds_nominal", robot_hw_nh,
                                      {20.0, 20.0, 20.0, 25.0, 25.0, 25.0});
  std::copy(thresholds.begin(), thresholds.end(),
            collision_config_.upper_force_thresholds_nominal.begin());

  return true;
}

void FrankaHW::update(const franka::RobotState& robot_state) {
  std::lock_guard<std::mutex> ros_lock(ros_state_mutex_);
  robot_state_ros_ = robot_state;
}

bool FrankaHW::controllerActive() const noexcept {
  return controller_active_;
}

void FrankaHW::control(
    const std::function<bool(const ros::Time&, const ros::Duration&)>& ros_callback) const {
  if (!initialized_) {
    ROS_ERROR("FrankaHW: Call to control before initialization!");
    return;
  }
  if (!controller_active_) {
    return;
  }

  franka::Duration last_time = robot_state_ros_.time;

  run_function_(*robot_, [this, ros_callback, &last_time](const franka::RobotState& robot_state,
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
void FrankaHW::doSwitch(const std::list<hardware_interface::ControllerInfo>& /* start_list */,
                        const std::list<hardware_interface::ControllerInfo>& /* stop_list */) {
  if (current_control_mode_ != ControlMode::None) {
    reset();
    controller_active_ = true;
  }
}

// prepareSwitch runs on the background message handling thread.
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

std::array<double, 7> FrankaHW::getJointPositionCommand() const noexcept {
  return position_joint_command_ros_.q;
}

std::array<double, 7> FrankaHW::getJointVelocityCommand() const noexcept {
  return velocity_joint_command_ros_.dq;
}

std::array<double, 7> FrankaHW::getJointEffortCommand() const noexcept {
  return effort_joint_command_ros_.tau_J;
}

void FrankaHW::reset() {
  position_joint_limit_interface_.reset();
}

void FrankaHW::checkJointLimits() {
  std::string joint_limits_warning;
  for (const auto& k_joint_name : joint_names_) {
    try {
      auto joint_handle = joint_state_interface_.getHandle(k_joint_name);
      auto urdf_joint = urdf_model_.getJoint(k_joint_name);
      joint_limits_interface::JointLimits joint_limits;
      if (getJointLimits(urdf_joint, joint_limits)) {
        double joint_lower = joint_limits.min_position;
        double joint_upper = joint_limits.max_position;
        double joint_position = joint_handle.getPosition();
        double dist = fmin(fabs(joint_position - joint_lower), fabs(joint_position - joint_upper));
        if (dist < joint_limit_warning_threshold_) {
          joint_limits_warning +=
              "\n\t" + k_joint_name + ": " + toStringWithPrecision(dist * 180 / M_PI) +
              " degrees to joint limits (limits: [" + toStringWithPrecision(joint_lower) + ", " +
              toStringWithPrecision(joint_upper) + "]" +
              " q: " + toStringWithPrecision(joint_position) + ") ";
        }
      } else {
        ROS_ERROR_STREAM_ONCE("FrankaHW: Could not parse joint limit for joint "
                              << k_joint_name << " for joint limit interfaces");
      }
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM_ONCE("FrankaHW: Could not get joint handle " << k_joint_name << " .\n"
                                                                    << ex.what());
      return;
    }
  }
  if (!joint_limits_warning.empty()) {
    ROS_WARN_THROTTLE(5, "FrankaHW: %s", joint_limits_warning.c_str());
  }
}

franka::Robot& FrankaHW::robot() const {
  if (!initialized_ || !robot_) {
    std::string error_message = "FrankaHW: Attempt to access robot before initialization!";
    ROS_ERROR("%s", error_message.c_str());
    throw std::logic_error(error_message);
  }
  return *robot_;
}

void FrankaHW::read(const ros::Time& /*time*/, const ros::Duration& /*period*/) {
  std::lock_guard<std::mutex> ros_lock(ros_state_mutex_);
  std::lock_guard<std::mutex> libfranka_lock(libfranka_state_mutex_);
  robot_state_ros_ = robot_state_libfranka_;
}

void FrankaHW::write(const ros::Time& /*time*/, const ros::Duration& /*period*/) {
  std::lock_guard<std::mutex> ros_lock(ros_cmd_mutex_);
  std::lock_guard<std::mutex> libfranka_lock(libfranka_cmd_mutex_);
  pose_cartesian_command_libfranka_ = pose_cartesian_command_ros_;
  velocity_cartesian_command_libfranka_ = velocity_cartesian_command_ros_;
  effort_joint_command_libfranka_ = effort_joint_command_ros_;
  position_joint_command_libfranka_ = position_joint_command_ros_;
  velocity_joint_command_libfranka_ = velocity_joint_command_ros_;
}

void FrankaHW::setupJointStateInterface(franka::RobotState& robot_state) {
  for (size_t i = 0; i < joint_names_.size(); i++) {
    hardware_interface::JointStateHandle joint_handle_q(joint_names_[i], &robot_state.q[i],
                                                        &robot_state.dq[i], &robot_state.tau_J[i]);
    joint_state_interface_.registerHandle(joint_handle_q);
  }
  registerInterface(&joint_state_interface_);
}

void FrankaHW::setupFrankaStateInterface(franka::RobotState& robot_state) {
  FrankaStateHandle franka_state_handle(arm_id_ + "_robot", robot_state);
  franka_state_interface_.registerHandle(franka_state_handle);
  registerInterface(&franka_state_interface_);
}

void FrankaHW::setupFrankaCartesianPoseInterface(franka::CartesianPose& pose_cartesian_command) {
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

void FrankaHW::setupFrankaModelInterface(franka::RobotState& robot_state) {
  if (model_) {
    franka_hw::FrankaModelHandle model_handle(arm_id_ + "_model", *model_, robot_state);
    franka_model_interface_.registerHandle(model_handle);
    registerInterface(&franka_model_interface_);
  }
}

bool FrankaHW::setRunFunction(const ControlMode& requested_control_mode,
                              const bool limit_rate,
                              const double cutoff_frequency,
                              const franka::ControllerMode internal_controller) {
  using std::placeholders::_1;
  using std::placeholders::_2;
  using Callback = std::function<bool(const franka::RobotState&, franka::Duration)>;

  switch (requested_control_mode) {
    case ControlMode::None:
      break;
    case ControlMode::JointTorque:
      run_function_ = [=](franka::Robot& robot, Callback ros_callback) {
        robot.control(std::bind(&FrankaHW::controlCallback<franka::Torques>, this,
                                std::cref(effort_joint_command_libfranka_), ros_callback, _1, _2),
                      limit_rate, cutoff_frequency);
      };
      break;
    case ControlMode::JointPosition:
      run_function_ = [=](franka::Robot& robot, Callback ros_callback) {
        robot.control(std::bind(&FrankaHW::controlCallback<franka::JointPositions>, this,
                                std::cref(position_joint_command_libfranka_), ros_callback, _1, _2),
                      internal_controller, limit_rate, cutoff_frequency);
      };
      break;
    case ControlMode::JointVelocity:
      run_function_ = [=](franka::Robot& robot, Callback ros_callback) {
        robot.control(std::bind(&FrankaHW::controlCallback<franka::JointVelocities>, this,
                                std::cref(velocity_joint_command_libfranka_), ros_callback, _1, _2),
                      internal_controller, limit_rate, cutoff_frequency);
      };
      break;
    case ControlMode::CartesianPose:
      run_function_ = [=](franka::Robot& robot, Callback ros_callback) {
        robot.control(std::bind(&FrankaHW::controlCallback<franka::CartesianPose>, this,
                                std::cref(pose_cartesian_command_libfranka_), ros_callback, _1, _2),
                      internal_controller, limit_rate, cutoff_frequency);
      };
      break;
    case ControlMode::CartesianVelocity:
      run_function_ = [=](franka::Robot& robot, Callback ros_callback) {
        robot.control(
            std::bind(&FrankaHW::controlCallback<franka::CartesianVelocities>, this,
                      std::cref(velocity_cartesian_command_libfranka_), ros_callback, _1, _2),
            internal_controller, limit_rate, cutoff_frequency);
      };
      break;
    case (ControlMode::JointTorque | ControlMode::JointPosition):
      run_function_ = [=](franka::Robot& robot, Callback ros_callback) {
        robot.control(std::bind(&FrankaHW::controlCallback<franka::Torques>, this,
                                std::cref(effort_joint_command_libfranka_), ros_callback, _1, _2),
                      std::bind(&FrankaHW::controlCallback<franka::JointPositions>, this,
                                std::cref(position_joint_command_libfranka_), ros_callback, _1, _2),
                      limit_rate, cutoff_frequency);
      };
      break;
    case (ControlMode::JointTorque | ControlMode::JointVelocity):
      run_function_ = [=](franka::Robot& robot, Callback ros_callback) {
        robot.control(std::bind(&FrankaHW::controlCallback<franka::Torques>, this,
                                std::cref(effort_joint_command_libfranka_), ros_callback, _1, _2),
                      std::bind(&FrankaHW::controlCallback<franka::JointVelocities>, this,
                                std::cref(velocity_joint_command_libfranka_), ros_callback, _1, _2),
                      limit_rate, cutoff_frequency);
      };
      break;
    case (ControlMode::JointTorque | ControlMode::CartesianPose):
      run_function_ = [=](franka::Robot& robot, Callback ros_callback) {
        robot.control(std::bind(&FrankaHW::controlCallback<franka::Torques>, this,
                                std::cref(effort_joint_command_libfranka_), ros_callback, _1, _2),
                      std::bind(&FrankaHW::controlCallback<franka::CartesianPose>, this,
                                std::cref(pose_cartesian_command_libfranka_), ros_callback, _1, _2),
                      limit_rate, cutoff_frequency);
      };
      break;
    case (ControlMode::JointTorque | ControlMode::CartesianVelocity):
      run_function_ = [=](franka::Robot& robot, Callback ros_callback) {
        robot.control(
            std::bind(&FrankaHW::controlCallback<franka::Torques>, this,
                      std::cref(effort_joint_command_libfranka_), ros_callback, _1, _2),
            std::bind(&FrankaHW::controlCallback<franka::CartesianVelocities>, this,
                      std::cref(velocity_cartesian_command_libfranka_), ros_callback, _1, _2),
            limit_rate, cutoff_frequency);
      };
      break;
    default:
      ROS_WARN("FrankaHW: No valid control mode selected; cannot switch controllers.");
      return false;
  }
  return true;
}

void FrankaHW::initROSInterfaces(ros::NodeHandle& /*robot_hw_nh*/) {
  setupJointStateInterface(robot_state_ros_);
  setupJointCommandInterface(position_joint_command_ros_.q, robot_state_ros_, true,
                             position_joint_interface_);
  setupJointCommandInterface(velocity_joint_command_ros_.dq, robot_state_ros_, true,
                             velocity_joint_interface_);
  setupJointCommandInterface(effort_joint_command_ros_.tau_J, robot_state_ros_, false,
                             effort_joint_interface_);
  setupLimitInterface<joint_limits_interface::PositionJointSoftLimitsHandle>(
      position_joint_limit_interface_, position_joint_interface_);
  setupLimitInterface<joint_limits_interface::VelocityJointSoftLimitsHandle>(
      velocity_joint_limit_interface_, velocity_joint_interface_);
  setupLimitInterface<joint_limits_interface::EffortJointSoftLimitsHandle>(
      effort_joint_limit_interface_, effort_joint_interface_);
  setupFrankaStateInterface(robot_state_ros_);
  setupFrankaCartesianPoseInterface(pose_cartesian_command_ros_);
  setupFrankaCartesianVelocityInterface(velocity_cartesian_command_ros_);
  setupFrankaModelInterface(robot_state_ros_);
}

void FrankaHW::initRobot() {
  robot_ = std::make_unique<franka::Robot>(robot_ip_, realtime_config_);
  model_ = std::make_unique<franka::Model>(robot_->loadModel());
  robot_->setCollisionBehavior(collision_config_.lower_torque_thresholds_acceleration,
                               collision_config_.upper_torque_thresholds_acceleration,
                               collision_config_.lower_torque_thresholds_nominal,
                               collision_config_.upper_torque_thresholds_nominal,
                               collision_config_.lower_force_thresholds_acceleration,
                               collision_config_.upper_force_thresholds_acceleration,
                               collision_config_.lower_force_thresholds_nominal,
                               collision_config_.upper_force_thresholds_nominal);
  update(robot_->readOnce());
}

void FrankaHW::setupParameterCallbacks(ros::NodeHandle& robot_hw_nh) {
  get_limit_rate_ = [robot_hw_nh]() {
    bool rate_limiting;
    robot_hw_nh.getParamCached("rate_limiting", rate_limiting);
    return rate_limiting;
  };

  get_internal_controller_ = [robot_hw_nh]() {
    std::string internal_controller;
    robot_hw_nh.getParamCached("internal_controller", internal_controller);
    franka::ControllerMode controller_mode;
    if (internal_controller == "joint_impedance") {
      controller_mode = franka::ControllerMode::kJointImpedance;
    } else if (internal_controller == "cartesian_impedance") {
      controller_mode = franka::ControllerMode::kCartesianImpedance;
    } else {
      ROS_WARN("Invalid internal_controller parameter provided, falling back to joint impedance");
      controller_mode = franka::ControllerMode::kJointImpedance;
    }
    return controller_mode;
  };

  get_cutoff_frequency_ = [robot_hw_nh]() {
    double cutoff_frequency;
    robot_hw_nh.getParamCached("cutoff_frequency", cutoff_frequency);
    return cutoff_frequency;
  };
}

bool FrankaHW::commandHasNaN(const franka::Torques& command) {
  return arrayHasNaN(command.tau_J);
}

bool FrankaHW::commandHasNaN(const franka::JointPositions& command) {
  return arrayHasNaN(command.q);
}

bool FrankaHW::commandHasNaN(const franka::JointVelocities& command) {
  return arrayHasNaN(command.dq);
}

bool FrankaHW::commandHasNaN(const franka::CartesianPose& command) {
  return arrayHasNaN(command.elbow) || arrayHasNaN(command.O_T_EE);
}

bool FrankaHW::commandHasNaN(const franka::CartesianVelocities& command) {
  return arrayHasNaN(command.elbow) || arrayHasNaN(command.O_dP_EE);
}

std::vector<double> FrankaHW::getCollisionThresholds(const std::string& name,
                                                     ros::NodeHandle& robot_hw_nh,
                                                     const std::vector<double>& defaults) {
  std::vector<double> thresholds;
  if (!robot_hw_nh.getParam("collision_config/" + name, thresholds) ||
      thresholds.size() != defaults.size()) {
    std::string message;
    for (const double& threshold : defaults) {
      message += std::to_string(threshold);
      message += " ";
    }
    ROS_INFO("No parameter %s found, using default values: %s", name.c_str(), message.c_str());
    return defaults;
  }
  return thresholds;
}

}  // namespace franka_hw
