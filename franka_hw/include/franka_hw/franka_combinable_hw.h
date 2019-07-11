// Copyright (c) 2019 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <atomic>
#include <functional>
#include <memory>
#include <string>
#include <thread>

#include <franka/exception.h>
#include <franka/robot_state.h>

#include <actionlib/server/simple_action_server.h>
#include <franka_hw/franka_hw.h>
#include <franka_hw/services.h>
#include <franka_msgs/ErrorRecoveryAction.h>
#include <ros/ros.h>

namespace franka_hw {

class FrankaCombinableHW : public FrankaHW {
 public:
  /**
   * Creates an instance of FrankaCombinableHW.
   *
   */
  FrankaCombinableHW();

  // FrankaHW::initParameters ()
  // FrankaHW::init()

  void initROSInterfaces(ros::NodeHandle& robot_hw_nh) override;
  void initRobot() override;

  void publishErrorState(const bool error);
  // FrankaH::read()
  // FrankaH::write()
  // void FrankaHW::checkJointLimits() {
  // FrankaHW::update(robot_state)
  // FrankaHW::controllerActive();
  void controlLoop();
  void setupServicesAndActionServers(ros::NodeHandle& node_handle);
  void control(
      const std::function<bool(const ros::Time&, const ros::Duration&)>& ros_callback) override;
  // FrankaHW::enforceLimits
  bool checkForConflict(const std::list<hardware_interface::ControllerInfo>& info) const override;
  // FrankaHW::doSwitch
  // FrankaHW::PrepareSwitch

  void read(const ros::Time&,                // NOLINT (readability-identifier-naming)
            const ros::Duration&) override;  // NOLINT [readability-named-parameter]
  void write(const ros::Time&,               // NOLINT (readability-identifier-naming)
             const ros::Duration& period) override;

  static std::array<double, 7> saturateTorqueRate(const std::array<double, 7>& tau_d_calculated,
                                                  const std::array<double, 7>& tau_J_d);

  // FrankaHW::getJointEffortCommand
  std::string getArmID();  // move into FrankaHW ??
  void triggerError();
  bool hasError();
  void resetError();
  bool controllerNeedsReset();
  // FrankaHW::CommandHasNaN methods

 private:
  template <typename T>
  T libfrankaUpdateCallback(const T& command,
                            const franka::RobotState& robot_state,
                            franka::Duration time_step) {
    if (commandHasNaN(command)) {
      std::string error_message = "FrankaCombinableHW: Got NaN value in command!";
      ROS_FATAL("%s", error_message.c_str());
      throw std::invalid_argument(error_message);
    }
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

  // FrankaHW::setupLimitInterface
  // FrankaHW::setupJointStateInterface
  // FrankaHW::setupCommandInterface
  // FrankaHW::setupFrankaStateInterface(franka::RobotState& robot_state);
  // setup other interfaces ...
  bool setRunFunction(const ControlMode& requested_control_mode,
                      const bool limit_rate,
                      const double cutoff_frequency,
                      const franka::ControllerMode internal_controller) override;
  // FrankaHW::setupParameterCallbacks
  // all interfaces from FrankaHW
  // all mutexes and data containers for commands and states

  // robot
  // model
  // runfunction but with Callback !!
  std::unique_ptr<std::thread> control_loop_thread_;
  ServiceContainer services_;
  std::unique_ptr<actionlib::SimpleActionServer<franka_msgs::ErrorRecoveryAction>>
      recovery_action_server_;
  std::atomic_bool has_error_{false};
  ros::Publisher has_error_pub_;
  std::atomic_bool error_recovered_{false};
  std::atomic_bool controller_needs_reset_{false};
};

}  // namespace franka_hw
