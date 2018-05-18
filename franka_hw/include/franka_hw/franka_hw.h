// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <array>
#include <atomic>
#include <functional>
#include <string>

#include <franka/duration.h>
#include <franka/model.h>
#include <franka/robot.h>
#include <franka/robot_state.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <ros/node_handle.h>
#include <ros/time.h>

#include <franka_hw/control_mode.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>

namespace franka_hw {

class FrankaHW : public hardware_interface::RobotHW {
 public:
  /**
   * Maximum allowed joint acceleration.
   *
   * Used to limit commanded joint positions, velocities, and torques.
   */
  static constexpr double kMaximumJointAcceleration{1.0};

  /**
   * Maximum allowed joint jerk.
   *
   * Used to limit commanded joint positions, velocities, and torques.
   */
  static constexpr double kMaximumJointJerk{4000.0};

  FrankaHW() = delete;

  /**
   * Creates an instance of FrankaHW that does not provide a model interface.
   *
   * @param[in] joint_names An array of joint names being controlled.
   * @param[in] arm_id Unique identifier for the robot being controlled.
   * @param[in] node_handle A node handle to get parameters from.
   */
  FrankaHW(const std::array<std::string, 7>& joint_names,
           const std::string& arm_id,
           const ros::NodeHandle& node_handle);

  /**
   * Creates an instance of FrankaHW that provides a model interface.
   *
   * @param[in] joint_names An array of joint names being controlled.
   * @param[in] arm_id Unique identifier for the robot being controlled.
   * @param[in] node_handle A node handle to get parameters from.
   * @param[in] model Robot model.
   */
  FrankaHW(const std::array<std::string, 7>& joint_names,
           const std::string& arm_id,
           const ros::NodeHandle& node_handle,
           franka::Model& model);

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
  void control(franka::Robot& robot,
               std::function<bool(const ros::Time&, const ros::Duration&)> ros_callback);

  /**
   * Updates the controller interfaces from the given robot state.
   *
   * @param[in] robot_state Current robot state.
   *
   * @throw franka::InvalidOperationException if a conflicting operation is already running.
   * @throw franka::NetworkException if the connection is lost.
   */
  void update(const franka::RobotState& robot_state);

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
  bool checkForConflict(const std::list<hardware_interface::ControllerInfo>& info) const override;

  /**
   * Performs controller switching (real-time capable).
   */
  void doSwitch(const std::list<hardware_interface::ControllerInfo>&,
                const std::list<hardware_interface::ControllerInfo>&) override;

  /**
  * Prepares switching between controllers (not real-time capable).
  *
  * @param[in] start_list Controllers requested to be started.
  * @param[in] stop_list Controllers requested to be stopped.
  *
  * @return True if the preparation has been successful, false otherwise.
  */
  bool prepareSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                     const std::list<hardware_interface::ControllerInfo>& stop_list) override;

  /**
   * Gets the current joint position command.
   *
   * @return Current joint position command.
   */
  std::array<double, 7> getJointPositionCommand() const noexcept;

  /**
   * Gets the current joint velocity command.
   *
   * @return Current joint velocity command.
   */
  std::array<double, 7> getJointVelocityCommand() const noexcept;

  /**
   * Gets the current joint torque command.
   *
   * @return Current joint torque command.
   */
  std::array<double, 7> getJointEffortCommand() const noexcept;

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
  using Callback = std::function<bool(const franka::RobotState&, franka::Duration)>;

  template <typename T>
  T controlCallback(const T& command,
                    Callback ros_callback,
                    const franka::RobotState& robot_state,
                    franka::Duration time_step) {
    robot_state_ = robot_state;
    if (!controller_active_ || ros_callback && !ros_callback(robot_state, time_step)) {
      return franka::MotionFinished(command);
    }
    return command;
  }

  hardware_interface::JointStateInterface joint_state_interface_{};
  franka_hw::FrankaStateInterface franka_state_interface_{};
  hardware_interface::PositionJointInterface position_joint_interface_{};
  hardware_interface::VelocityJointInterface velocity_joint_interface_{};
  hardware_interface::EffortJointInterface effort_joint_interface_{};
  franka_hw::FrankaPoseCartesianInterface franka_pose_cartesian_interface_{};
  franka_hw::FrankaVelocityCartesianInterface franka_velocity_cartesian_interface_{};
  franka_hw::FrankaModelInterface franka_model_interface_{};

  joint_limits_interface::PositionJointSoftLimitsInterface position_joint_limit_interface_{};
  joint_limits_interface::VelocityJointSoftLimitsInterface velocity_joint_limit_interface_{};
  joint_limits_interface::EffortJointSoftLimitsInterface effort_joint_limit_interface_{};

  franka::RobotState robot_state_{};

  std::array<std::string, 7> joint_names_;
  const std::string arm_id_;

  franka::JointPositions position_joint_command_;
  franka::JointVelocities velocity_joint_command_;
  franka::Torques effort_joint_command_;
  franka::CartesianPose pose_cartesian_command_;
  franka::CartesianVelocities velocity_cartesian_command_;

  std::function<void(franka::Robot&, Callback)> run_function_;

  std::atomic_bool controller_active_{false};
  ControlMode current_control_mode_ = ControlMode::None;
};

}  // namespace franka_hw
