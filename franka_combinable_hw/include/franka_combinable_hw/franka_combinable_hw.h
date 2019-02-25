// Copyright (c) 2019 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <array>
#include <atomic>
#include <exception>
#include <functional>
#include <string>
#include <thread>

#include <actionlib/server/simple_action_server.h>
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
#include <urdf/model.h>

#include <franka_control/ErrorRecoveryAction.h>
#include <franka_control/services.h>
#include <franka_hw/control_mode.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>

using franka_control::ServiceContainer;
using franka_hw::ControlMode;

namespace franka_combinable_hw {

class FrankaCombinableHW : public hardware_interface::RobotHW {
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

  /**
   * Creates an instance of FrankaHW
   *
   */
  FrankaCombinableHW();

  /**
   * Creates an instance of FrankaCombinableHW that provides a model interface.
   *
   * @param[in] root_nh A NodeHandle in the root of the caller namespace.
   * @param[in] robot_hw_nh A NodeHandle in the namespace from which the RobotHW
   * should read its configuration.
   *
   * @return True if initialization was successful
   */
  bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) override;

  ~FrankaCombinableHW() override = default;

  /**
   * Runs the currently active controller in a realtime loop.
   *
   * If no controller is active, the function immediately exits. When running a controller,
   * the function only exits when ros_callback returns false.
   *
   * @param[in] robot Robot instance.
   *
   * @throw franka::ControlException if an error related to torque control or motion generation
   * occurred.
   * @throw franka::InvalidOperationException if a conflicting operation is already running.
   * @throw franka::NetworkException if the connection is lost, e.g. after a timeout.
   * @throw franka::RealtimeException if realtime priority cannot be set for the current thread.
   */
  void control(franka::Robot& robot);

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
   * Reads data from the franka robot
   *
   * \param time The current time
   * \param period The time passed since the last call to \ref read
   */
  void read(const ros::Time& time, const ros::Duration& period) override;

  /**
   * Writes data to the franka robot
   *
   * \param time The current time
   * \param period The time passed since the last call to \ref write
   */
  void write(const ros::Time& time, const ros::Duration& period) override;

  /**
   * Gets the current joint torque command.
   *
   * @return Current joint torque command.
   */
  std::array<double, 7> getJointEffortCommand() const noexcept;

  /**
   * Enforces limits on torque level.
   *
   * @param[in] period Duration of the current cycle.
   */
  void enforceLimits(const ros::Duration& period);

  /**
   * Gets arm id.
   */
  std::string getArmID();

  /**
   * Trigger error state to stop controller.
   */
  void triggerError();

  /**
   * Returns error state.
   *
   * @return true if there is currently error.
   */
  bool hasError();

  /**
   * Reset error state.
   */
  void resetError();

  /**
   * Returns whether the controller needs to be reset.
   */
  bool controllerNeedsReset();

 private:
  using Callback = std::function<bool(const franka::RobotState&, franka::Duration)>;

  template <typename T>
  T controlCallback(const T& command,
                    const franka::RobotState& robot_state,
                    franka::Duration time_step) {
    checkJointLimits();

    libfranka_state_mutex_.lock();
    robot_state_libfranka_ = robot_state;
    libfranka_state_mutex_.unlock();

    libfranka_cmd_mutex_.lock();
    T current_cmd = command;
    libfranka_cmd_mutex_.unlock();

    if (has_error_ || !controller_active_) {
      return franka::MotionFinished(current_cmd);
    }
    return current_cmd;
  }

  void controlLoop();

  void setupServicesAndActionServers(ros::NodeHandle& node_handle);

  static std::array<double, 7> saturateTorqueRate(const std::array<double, 7>& tau_d_calculated,
                                                  const std::array<double, 7>& tau_J_d);

  bool initRobot(ros::NodeHandle& robot_hw_nh);

  void publishErrorState(bool error);

  void checkJointLimits();

  std::unique_ptr<franka::Robot> robot_;
  std::unique_ptr<franka::Model> model_;

  hardware_interface::JointStateInterface joint_state_interface_{};
  franka_hw::FrankaStateInterface franka_state_interface_{};
  hardware_interface::EffortJointInterface effort_joint_interface_{};
  franka_hw::FrankaPoseCartesianInterface franka_pose_cartesian_interface_{};
  franka_hw::FrankaVelocityCartesianInterface franka_velocity_cartesian_interface_{};
  franka_hw::FrankaModelInterface franka_model_interface_{};

  joint_limits_interface::EffortJointSoftLimitsInterface effort_joint_limit_interface_{};

  urdf::Model urdf_model_;

  franka::RobotState robot_state_libfranka_{};
  franka::RobotState robot_state_ros_{};

  std::mutex libfranka_state_mutex_;
  std::mutex ros_state_mutex_;

  std::array<std::string, 7> joint_names_;
  std::string arm_id_;
  std::string robot_ip_;

  // command data of libfranka
  std::mutex libfranka_cmd_mutex_;
  franka::Torques effort_joint_command_libfranka_;
  franka::CartesianPose pose_cartesian_command_libfranka_;
  franka::CartesianVelocities velocity_cartesian_command_libfranka_;

  // command data of frankahw ros
  std::mutex ros_cmd_mutex_;
  franka::Torques effort_joint_command_ros_;
  franka::CartesianPose pose_cartesian_command_ros_;
  franka::CartesianVelocities velocity_cartesian_command_ros_;

  std::function<void(franka::Robot&)> run_function_;

  bool limit_rate_{true};
  std::atomic_bool controller_active_{false};
  ControlMode current_control_mode_ = ControlMode::None;
  double joint_limit_warning_threshold_{10 * 3.14 / 180};

  std::unique_ptr<std::thread> control_loop_thread_;

  ros::Publisher has_error_pub_;
  ServiceContainer services_;
  std::unique_ptr<actionlib::SimpleActionServer<franka_control::ErrorRecoveryAction>>
      recovery_action_server_;

  std::atomic<bool> has_error_;
  std::atomic<bool> error_recovered_;
  bool controller_needs_reset_;
  bool initialized_ = false;
};

}  // namespace franka_combinable_hw
