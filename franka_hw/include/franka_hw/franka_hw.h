// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <array>
#include <atomic>
#include <exception>
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
  /**
   * Default constructor. Note: Using this constructor you will have to call init before operation.
   */
  FrankaHW();

  virtual ~FrankaHW() override = default;

  /**
   * TODO(jaeh_ch)
   *
   */
  virtual bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) override;

  /**
   * Initializes the Parameterization of the FrankaHW class from the ROS parameter server and
   * sets up interfaces for ros_control.
   *  TODO(jaeh_ch): explain with parameters required from parameter server
   *
   * @return True if initialization was successful, false otherwise.
   */
  virtual void initROSInterfaces(ros::NodeHandle& robot_hw_nh);

  /**
   * Runs the currently active controller in a realtime loop.
   *
   * If no controller is active, the function immediately exits. When running a controller,
   * the function only exits when ros_callback returns false.
   *
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

  /**
   * Reads data from the franka robot.
   *
   * @param[in] time The current time.
   * @param[in] period The time passed since the last call to \ref read.
   */
  void read(const ros::Time& time, const ros::Duration& period) override;

  /**
   * Writes data to the franka robot.
   *
   * @param[in] time The current time.
   * @param[in] period The time passed since the last call to \ref write.
   */
  void write(const ros::Time& time, const ros::Duration& period) override;

  /** TODO
   */
  franka::Robot& robot();

  franka::ControllerMode getInternalController();

  double getCutoffFrequency();

  bool getRateLimiting();

  static bool commandHasNaN(const franka::Torques& command);
  static bool commandHasNaN(const franka::JointPositions& command);
  static bool commandHasNaN(const franka::JointVelocities& command);
  static bool commandHasNaN(const franka::CartesianPose& command);
  static bool commandHasNaN(const franka::CartesianVelocities& command);

 protected:
  template <size_t size>
  static bool arrayHasNaN(const std::array<double, size> array) {
    return std::any_of(array.begin(), array.end(), [](const double& e) { return std::isnan(e); });
  }

  using Callback = std::function<bool(const franka::RobotState&, franka::Duration)>;
  template <typename T>
  T controlCallback(const T& command,
                    Callback ros_callback,
                    const franka::RobotState& robot_state,
                    franka::Duration time_step) {
    robot_state_libfranka_ = robot_state;
    ros::Time now = ros::Time(0);
    read(now, ros::Duration(time_step.toSec()));

    if (!controller_active_ || (ros_callback && !ros_callback(robot_state, time_step))) {
      return franka::MotionFinished(command);
    }

    write(now, ros::Duration(time_step.toSec()));
    if (commandHasNaN(command)) {
      std::string error_message = "FrankaHW::controlCallback: Got NaN command!";
      ROS_FATAL("%s", error_message.c_str());
      throw std::invalid_argument(error_message);
    }

    return command;
  }

  template <typename T>
  void setupLimitInterface(joint_limits_interface::JointLimitsInterface<T>& limit_interface,
                           hardware_interface::JointCommandInterface& command_interface) {
    joint_limits_interface::SoftJointLimits soft_limits;
    joint_limits_interface::JointLimits joint_limits;
    for (size_t i = 0; i < joint_names_.size(); i++) {
      const std::string& joint_name = joint_names_[i];
      auto urdf_joint = urdf_model_.getJoint(joint_name);
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

  template <typename T>
  void setupJointCommandInterface(std::array<double, 7>& command,
                                  franka::RobotState& state,
                                  const bool use_q_d,
                                  T& interface) {
    for (size_t i = 0; i < joint_names_.size(); i++) {
      hardware_interface::JointStateHandle state_handle;
      if (use_q_d) {
        state_handle = hardware_interface::JointStateHandle(joint_names_[i], &state.q_d[i],
                                                            &state.dq_d[i], &state.tau_J[i]);
      } else {
        state_handle = hardware_interface::JointStateHandle(joint_names_[i], &state.q[i],
                                                            &state.dq[i], &state.tau_J[i]);
      }
      hardware_interface::JointHandle joint_handle(state_handle, &command[i]);
      interface.registerHandle(joint_handle);
    }
    registerInterface(&interface);
  }

  virtual void setupFrankaStateInterface(franka::RobotState& robot_state);

  virtual void setupFrankaCartesianPoseInterface(franka::CartesianPose& pose_cartesian_command);

  virtual void setupFrankaCartesianVelocityInterface(
      franka::CartesianVelocities& velocity_cartesian_command);

  virtual void setupFrankaModelInterface(franka::RobotState& robot_state);

  virtual bool setRunFunction(const ControlMode& requested_control_mode,
                              const bool limit_rate,
                              const double cutoff_frequency,
                              const franka::ControllerMode internal_controller);

  virtual bool initParameters(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh);

  virtual void initRobot();

  virtual void setupParameterCallbacks(ros::NodeHandle& robot_hw_nh);

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

  std::mutex libfranka_state_mutex_;
  std::mutex ros_state_mutex_;
  franka::RobotState robot_state_libfranka_{};
  franka::RobotState robot_state_ros_{};

  std::mutex libfranka_cmd_mutex_;
  franka::JointPositions position_joint_command_libfranka_;
  franka::JointVelocities velocity_joint_command_libfranka_;
  franka::Torques effort_joint_command_libfranka_;
  franka::CartesianPose pose_cartesian_command_libfranka_;
  franka::CartesianVelocities velocity_cartesian_command_libfranka_;

  std::mutex ros_cmd_mutex_;
  franka::JointPositions position_joint_command_ros_;
  franka::JointVelocities velocity_joint_command_ros_;
  franka::Torques effort_joint_command_ros_;
  franka::CartesianPose pose_cartesian_command_ros_;
  franka::CartesianVelocities velocity_cartesian_command_ros_;

  std::unique_ptr<franka::Robot> robot_;
  std::unique_ptr<franka::Model> model_;

  std::array<std::string, 7> joint_names_;
  std::string arm_id_;
  std::string robot_ip_;
  urdf::Model urdf_model_;
  double joint_limit_warning_threshold_{0.1};

  bool initialized_{false};
  std::atomic_bool controller_active_{false};
  ControlMode current_control_mode_ = ControlMode::None;

  std::function<franka::ControllerMode()> get_internal_controller_;
  std::function<bool()> get_limit_rate_;
  std::function<double()> get_cutoff_frequency_;
  std::function<void(franka::Robot&, Callback)> run_function_;
};

}  // namespace franka_hw
