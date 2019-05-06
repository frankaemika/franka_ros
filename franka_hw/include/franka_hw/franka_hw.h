// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <array>
#include <atomic>
#include <functional>
#include <list>
#include <string>

#include <franka/duration.h>
#include <franka/model.h>
#include <franka/rate_limiting.h>
#include <franka/robot.h>
#include <franka/robot_state.h>
#include <franka_hw/control_mode.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <franka_hw/resource_helpers.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <ros/time.h>
#include <urdf/model.h>

namespace franka_hw {

using std::string;
using std::vector;
using std::function;
using std::array;
using std::list;
using franka::RobotState;
using franka::ControllerMode;
using franka::kDefaultCutoffFrequency;
using franka::Robot;
using hardware_interface::ControllerInfo;
using hardware_interface::RobotHW;
using hardware_interface::JointStateInterface;
using hardware_interface::JointCommandInterface;
using hardware_interface::PositionJointInterface;
using hardware_interface::VelocityJointInterface;
using hardware_interface::EffortJointInterface;
using franka_hw::FrankaStateInterface;
using franka_hw::FrankaPoseCartesianInterface;
using franka_hw::FrankaVelocityCartesianInterface;
using franka_hw::FrankaModelInterface;
using joint_limits_interface::PositionJointSoftLimitsInterface;
using joint_limits_interface::VelocityJointSoftLimitsInterface;
using joint_limits_interface::EffortJointSoftLimitsInterface;
using joint_limits_interface::JointLimitsInterface;
using Callback = function<bool(const RobotState&, franka::Duration)>;

class FrankaHW : public hardware_interface::RobotHW {
 public:
  FrankaHW() = delete;

  /**
   * Creates an instance of FrankaHW that does not provide a model interface.
   *
   * @param[in] joint_names An array of joint names being controlled.
   * @param[in] arm_id Unique identifier for the robot being controlled.
   * @param[in] urdf_model A URDF model to initialize joint limits from.
   * @param[in] get_limit_rate Getter that should return true if the rate limiter
   * should be used, false otherwise. Defaults to true.
   * @param[in] get_cutoff_frequency Getter for cutoff frequency for the low-pass filter.
   * Defaults to franka::kDefaultCutoffFrequency.
   * @param[in] get_internal_controller Getter for an internal controller to
   * be used for control loops using only motion generation. Defaults to joint impedance.
   */
  FrankaHW(const array<string, 7>& joint_names,
           const string& arm_id,
           const urdf::Model& urdf_model,
           function<bool()> get_limit_rate = []() { return true; },
           function<double()> get_cutoff_frequency = []() { return kDefaultCutoffFrequency; },
           function<ControllerMode()> get_internal_controller =
               []() { return ControllerMode::kJointImpedance; });

  /**
   * Creates an instance of FrankaHW that provides a model interface.
   *
   * @param[in] joint_names An array of joint names being controlled.
   * @param[in] arm_id Unique identifier for the robot being controlled.
   * @param[in] urdf_model A URDF model to initialize joint limits from.
   * @param[in] model Robot model.
   * @param[in] get_limit_rate Getter that should return true if the rate limiter
   * should be used, false otherwise. Defaults to true.
   * @param[in] get_cutoff_frequency Getter for cutoff frequency for the low-pass filter.
   * Defaults to franka::kDefaultCutoffFrequency.
   * @param[in] get_internal_controller Getter for an internal controller to
   * be used for control loops using only motion generation. Defaults to joint impedance.
   */
  FrankaHW(const array<string, 7>& joint_names,
           const string& arm_id,
           const urdf::Model& urdf_model,
           franka::Model& model,
           function<bool()> get_limit_rate = []() { return true; },
           function<double()> get_cutoff_frequency = []() { return kDefaultCutoffFrequency; },
           function<ControllerMode()> get_internal_controller =
               []() { return ControllerMode::kJointImpedance; });

  ~FrankaHW() override = default;

  /**
   * Runs the currently active controller in a realtime loop.
   *
   * If no controller is active, the function immediately exits. When running a controller,
   * the function only exits when ros_callback returns false.
   *
   * @param[in] robot Robot instance.
   * @param[in] ros_callback A callback function that is executed at each time step and runs
   * all ROS-side functionality of the hardware. Execution is stopped if it returns false.
   *
   * @throw franka::ControlException if an error related to torque control or motion generation
   * occurred.
   * @throw franka::InvalidOperationException if a conflicting operation is already running.
   * @throw franka::NetworkException if the connection is lost, e.g. after a timeout.
   * @throw franka::RealtimeException if realtime priority cannot be set for the current thread.
   */
  void control(Robot& robot,
               const function<bool(const ros::Time&, const ros::Duration&)>& ros_callback);

  /**
   * Updates the controller interfaces from the given robot state.
   *
   * @param[in] robot_state Current robot state.
   *
   * @throw franka::InvalidOperationException if a conflicting operation is already running.
   * @throw franka::NetworkException if the connection is lost.
   */
  void update(const RobotState& robot_state);

  /**
   * Indicates whether there is an active controller.
   *
   * @return True if a controller is currently active, false otherwise.
   */
  bool controllerActive() const noexcept;

  /**
   * Checks whether a requested controller can be run, based on the resources
   * and interfaces it claims.
   *
   * @param[in] info Controllers to be running at the same time.
   *
   * @return True in case of a conflict, false in case of valid controllers.
   */
  bool checkForConflict(const list<ControllerInfo>& info) const override;

  /**
   * Performs controller switching (real-time capable).
   */
  void doSwitch(const list<ControllerInfo>&, const list<ControllerInfo>&) override;

  /**
   * Prepares switching between controllers (not real-time capable).
   *
   * @param[in] start_list Controllers requested to be started.
   * @param[in] stop_list Controllers requested to be stopped.
   *
   * @return True if the preparation has been successful, false otherwise.
   */
  bool prepareSwitch(const list<ControllerInfo>& start_list,
                     const list<ControllerInfo>& stop_list) override;

  /**
   * Gets the current joint position command.
   *
   * @return Current joint position command.
   */
  array<double, 7> getJointPositionCommand() const noexcept;

  /**
   * Gets the current joint velocity command.
   *
   * @return Current joint velocity command.
   */
  array<double, 7> getJointVelocityCommand() const noexcept;

  /**
   * Gets the current joint torque command.
   *
   * @return Current joint torque command.
   */
  array<double, 7> getJointEffortCommand() const noexcept;

  /**
   * Enforces limits on position, velocity, and torque level.
   *
   * @param[in] period Duration of the current cycle.
   */
  void enforceLimits(const ros::Duration& period);

  /**
   * Resets the limit interfaces.
   */
  void reset();

 private:
  template <typename T>
  T controlCallback(const T& command,
                    Callback ros_callback,
                    const RobotState& robot_state,
                    franka::Duration time_step) {
    robot_state_ = robot_state;
    if (!controller_active_ || (ros_callback && !ros_callback(robot_state, time_step))) {
      return franka::MotionFinished(command);
    }
    return command;
  }

  template <typename T>
  void setupLimitInterface(const urdf::Model& urdf_model,
                           JointLimitsInterface<T>& limit_interface,
                           JointCommandInterface& command_interface) {
    joint_limits_interface::SoftJointLimits soft_limits;
    joint_limits_interface::JointLimits joint_limits;
    for (size_t i = 0; i < joint_names_.size(); i++) {
      const std::string& joint_name = joint_names_[i];
      auto urdf_joint = urdf_model.getJoint(joint_name);
      if (!urdf_joint || !urdf_joint->safety || !urdf_joint->limits) {
        ROS_WARN(
            "FrankaHW: Joint %s has incomplete limits and safety specs. Skipping it in the joint "
            "limit interface!",
            joint_name.c_str());
        continue;
      }
      if (joint_limits_interface::getSoftJointLimits(urdf_joint, soft_limits)) {
        if (joint_limits_interface::getJointLimits(urdf_joint, joint_limits)) {
          joint_limits.max_acceleration = franka::kMaxJointAcceleration[i];
          joint_limits.has_acceleration_limits = true;
          joint_limits.max_jerk = franka::kMaxJointJerk[i];
          joint_limits.has_jerk_limits = true;
          T limit_handle(command_interface.getHandle(joint_name), joint_limits, soft_limits);
          limit_interface.registerHandle(limit_handle);
        } else {
          ROS_ERROR(
              "FrankaHW: Could not parse joint limit for joint: %s for joint limit interfaces",
              joint_name.c_str());
        }
      } else {
        ROS_ERROR(
            "FrankaHW: Could not parse soft joint limit for joint %s for joint limit interfaces",
            joint_name.c_str());
      }
    }
  }

  void setupJointStateInterface(RobotState& robot_state);

  void setupJointCommandInterface(franka::JointPositions& position_joint_command,
                                  RobotState& robot_state);
  void setupJointCommandInterface(franka::JointVelocities& velocity_joint_command,
                                  RobotState& robot_state);
  void setupJointCommandInterface(franka::Torques& effort_joint_command);

  void setupFrankaStateInterface(RobotState& robot_state);

  void setupFrankaCartesianPoseInterface(franka::CartesianPose& pose_cartesian_command);

  void setupFrankaCartesianVelocityInterface(
      franka::CartesianVelocities& velocity_cartesian_command);

  void setupFrankaModelInterface(franka::Model& model, RobotState& robot_state);

  bool setRunFunction(const ControlMode& requested_control_mode,
                      const bool limit_rate,
                      const double cutoff_frequency,
                      const franka::ControllerMode internal_controller);

  JointStateInterface joint_state_interface_{};
  FrankaStateInterface franka_state_interface_{};
  PositionJointInterface position_joint_interface_{};
  VelocityJointInterface velocity_joint_interface_{};
  EffortJointInterface effort_joint_interface_{};
  FrankaPoseCartesianInterface franka_pose_cartesian_interface_{};
  FrankaVelocityCartesianInterface franka_velocity_cartesian_interface_{};
  FrankaModelInterface franka_model_interface_{};
  PositionJointSoftLimitsInterface position_joint_limit_interface_{};
  VelocityJointSoftLimitsInterface velocity_joint_limit_interface_{};
  EffortJointSoftLimitsInterface effort_joint_limit_interface_{};

  RobotState robot_state_{};

  array<string, 7> joint_names_;
  const string arm_id_;
  function<ControllerMode()> get_internal_controller_;
  function<bool()> get_limit_rate_;
  function<double()> get_cutoff_frequency_;

  franka::JointPositions position_joint_command_;
  franka::JointVelocities velocity_joint_command_;
  franka::Torques effort_joint_command_;
  franka::CartesianPose pose_cartesian_command_;
  franka::CartesianVelocities velocity_cartesian_command_;

  function<void(Robot&, Callback)> run_function_;

  std::atomic_bool controller_active_{false};
  ControlMode current_control_mode_ = ControlMode::None;
};

}  // namespace franka_hw
