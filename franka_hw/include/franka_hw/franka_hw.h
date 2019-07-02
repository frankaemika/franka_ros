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
  FrankaHW(const std::array<std::string, 7>& joint_names,
           const std::string& arm_id,
           const urdf::Model& urdf_model,
           std::function<bool()> get_limit_rate = []() { return true; },
           std::function<double()> get_cutoff_frequency =
               []() { return franka::kDefaultCutoffFrequency; },
           std::function<franka::ControllerMode()> get_internal_controller =
               []() { return franka::ControllerMode::kJointImpedance; });

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
  FrankaHW(const std::array<std::string, 7>& joint_names,
           const std::string& arm_id,
           const urdf::Model& urdf_model,
           franka::Model& model,
           std::function<bool()> get_limit_rate = []() { return true; },
           std::function<double()> get_cutoff_frequency =
               []() { return franka::kDefaultCutoffFrequency; },
           std::function<franka::ControllerMode()> get_internal_controller =
               []() { return franka::ControllerMode::kJointImpedance; });

  virtual ~FrankaHW() override = default;

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
  virtual void control(
      franka::Robot& robot,
      const std::function<bool(const ros::Time&, const ros::Duration&)>& ros_callback);

  /**
   * Updates the controller interfaces from the given robot state.
   *
   * @param[in] robot_state Current robot state.
   *
   * @throw franka::InvalidOperationException if a conflicting operation is already running.
   * @throw franka::NetworkException if the connection is lost.
   */
  virtual void update(const franka::RobotState& robot_state);

  /**
   * Indicates whether there is an active controller.
   *
   * @return True if a controller is currently active, false otherwise.
   */
  virtual bool controllerActive() const noexcept;

  /**
   * Checks whether a requested controller can be run, based on the resources
   * and interfaces it claims.
   *
   * @param[in] info Controllers to be running at the same time.
   *
   * @return True in case of a conflict, false in case of valid controllers.
   */
  virtual bool checkForConflict(
      const std::list<hardware_interface::ControllerInfo>& info) const override;

  /**
   * Performs controller switching (real-time capable).
   */
  virtual void doSwitch(const std::list<hardware_interface::ControllerInfo>&,
                        const std::list<hardware_interface::ControllerInfo>&) override;

  /**
   * Prepares switching between controllers (not real-time capable).
   *
   * @param[in] start_list Controllers requested to be started.
   * @param[in] stop_list Controllers requested to be stopped.
   *
   * @return True if the preparation has been successful, false otherwise.
   */
  virtual bool prepareSwitch(
      const std::list<hardware_interface::ControllerInfo>& start_list,
      const std::list<hardware_interface::ControllerInfo>& stop_list) override;

  /**
   * Gets the current joint position command.
   *
   * @return Current joint position command.
   */
  virtual std::array<double, 7> getJointPositionCommand() const noexcept;

  /**
   * Gets the current joint velocity command.
   *
   * @return Current joint velocity command.
   */
  virtual std::array<double, 7> getJointVelocityCommand() const noexcept;

  /**
   * Gets the current joint torque command.
   *
   * @return Current joint torque command.
   */
  virtual std::array<double, 7> getJointEffortCommand() const noexcept;

  /**
   * Enforces limits on position, velocity, and torque level.
   *
   * @param[in] period Duration of the current cycle.
   */
  virtual void enforceLimits(const ros::Duration& period);

  /**
   * Resets the limit interfaces.
   */
  virtual void reset();

  /**
   * Checks the proximity of each joint to its joint position limits and prints
   * a warning whenever a joint is close to a limit.
   */
  virtual void checkJointLimits();

 protected:
  using Callback = std::function<bool(const franka::RobotState&, franka::Duration)>;
  template <typename T>
  T controlCallback(const T& command,
                    Callback ros_callback,
                    const franka::RobotState& robot_state,
                    franka::Duration time_step) {
    robot_state_ = robot_state;
    if (!controller_active_ || (ros_callback && !ros_callback(robot_state, time_step))) {
      return franka::MotionFinished(command);
    }
    return command;
  }

  template <typename T>
  void setupLimitInterface(const urdf::Model& urdf_model,
                           joint_limits_interface::JointLimitsInterface<T>& limit_interface,
                           hardware_interface::JointCommandInterface& command_interface) {
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

  virtual void setupJointStateInterface(franka::RobotState& robot_state);

  virtual void setupJointCommandInterface(franka::JointPositions& position_joint_command,
                                          franka::RobotState& robot_state);

  virtual void setupJointCommandInterface(franka::JointVelocities& velocity_joint_command,
                                          franka::RobotState& robot_state);

  virtual void setupJointCommandInterface(franka::Torques& effort_joint_command);

  virtual void setupFrankaStateInterface(franka::RobotState& robot_state);

  virtual void setupFrankaCartesianPoseInterface(franka::CartesianPose& pose_cartesian_command);

  virtual void setupFrankaCartesianVelocityInterface(
      franka::CartesianVelocities& velocity_cartesian_command);

  virtual void setupFrankaModelInterface(franka::Model& model, franka::RobotState& robot_state);

  virtual bool setRunFunction(const ControlMode& requested_control_mode,
                              const bool limit_rate,
                              const double cutoff_frequency,
                              const franka::ControllerMode internal_controller);

  virtual void init(const std::array<std::string, 7>& joint_names,
                    const std::string& arm_id,
                    const urdf::Model& urdf_model,
                    std::function<bool()> get_limit_rate = []() { return true; },
                    std::function<double()> get_cutoff_frequency =
                        []() { return franka::kDefaultCutoffFrequency; },
                    std::function<franka::ControllerMode()> get_internal_controller =
                        []() { return franka::ControllerMode::kJointImpedance; });

  hardware_interface::JointStateInterface joint_state_interface_{};
  FrankaStateInterface franka_state_interface_{};
  hardware_interface::PositionJointInterface position_joint_interface_{};
  hardware_interface::VelocityJointInterface velocity_joint_interface_{};
  hardware_interface::EffortJointInterface effort_joint_interface_{};
  FrankaPoseCartesianInterface franka_pose_cartesian_interface_{};
  FrankaVelocityCartesianInterface franka_velocity_cartesian_interface_{};
  FrankaModelInterface franka_model_interface_{};
  joint_limits_interface::PositionJointSoftLimitsInterface position_joint_limit_interface_{};
  joint_limits_interface::VelocityJointSoftLimitsInterface velocity_joint_limit_interface_{};
  joint_limits_interface::EffortJointSoftLimitsInterface effort_joint_limit_interface_{};

  franka::RobotState robot_state_{};

  std::array<std::string, 7> joint_names_;
  const std::string arm_id_;
  std::function<franka::ControllerMode()> get_internal_controller_;
  std::function<bool()> get_limit_rate_;
  std::function<double()> get_cutoff_frequency_;

  franka::JointPositions position_joint_command_;
  franka::JointVelocities velocity_joint_command_;
  franka::Torques effort_joint_command_;
  franka::CartesianPose pose_cartesian_command_;
  franka::CartesianVelocities velocity_cartesian_command_;

  std::function<void(franka::Robot&, Callback)> run_function_;

  urdf::Model urdf_model_;
  double joint_limit_warning_threshold_{0.1};

  std::atomic_bool controller_active_{false};
  ControlMode current_control_mode_ = ControlMode::None;
};

}  // namespace franka_hw
