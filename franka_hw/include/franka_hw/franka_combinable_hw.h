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

/**
 * A hardware class for a Panda robot based on the ros_control framework.
 * This class is ready to be combined with other hardware classes e.g. to
 * control multiple robots from a single controller.
 * Note: This class allows for torque (effort) control only due to the lack of synchronization
 * between master controllers of different robots. For more information see the documentation at
 * https://frankaemika.github.io/docs/franka_ros.html .
 */
class FrankaCombinableHW : public FrankaHW {
 public:
  /**
   * Creates an instance of FrankaCombinableHW.
   */
  FrankaCombinableHW();

  /**
   * Initializes the class in terms of ros_control interfaces.
   * Note: You have to call initParameters beforehand. Use the complete initialization routine
   * \ref init() method to control robots.
   *
   * @param[in] robot_hw_nh A node handle in the namespace of the robot hardware.
   * @return True if successful, false otherwise.
   */
  void initROSInterfaces(ros::NodeHandle& robot_hw_nh) override;

  /**
   * Runs the currently active controller in a realtime loop. If no controller is active, the
   * function immediately exits.
   *
   * @param[in] ros_callback A callback function that is executed at each time step.
   *
   * @throw franka::ControlException if an error related to torque control occurred.
   * @throw franka::InvalidOperationException if a conflicting operation is already running.
   * @throw franka::NetworkException if the connection is lost, e.g. after a timeout.
   * @throw franka::RealtimeException if realtime priority cannot be set for the current thread.
   */
  void control(const std::function<bool(const ros::Time&, const ros::Duration&)>&
                   ros_callback =  // NOLINT (google-default-arguments)
               [](const ros::Time&, const ros::Duration&) {
                 return true;
               }) const override;  // NOLINT (google-default-arguments)

  /**
   * Checks whether a requested controller can be run, based on the resources and interfaces it
   * claims. Note: FrankaCombinableHW allows torque control only.
   *
   * @param[in] info Controllers to be running at the same time.
   *
   * @return True in case of a conflict, false in case of valid controllers.
   */
  bool checkForConflict(const std::list<hardware_interface::ControllerInfo>& info) const override;

  /**
   * Reads data from the franka robot.
   *
   * @param[in] time The current time. Not used in this class.
   * @param[in] period The time passed since the last call to \ref read. Not used in this class.
   */
  void read(const ros::Time& /*time*/, const ros::Duration& /*period*/) override;

  /**
   * Writes data to the franka robot.
   *
   * @param[in] time The current time. Not used in this class.
   * @param[in] period The time passed since the last call to \ref write.
   */
  void write(const ros::Time& /*time*/, const ros::Duration& period) override;

  /**
   * Getter method for the arm_id which is used to distinguish between multiple
   * instances of FrankaCombinableHW.
   *
   * @return A copy of the arm_id string identifying the class instance.
   */
  std::string getArmID() const noexcept;

  /**
   * Triggers a stop of the controlLoop. This interface is used to stop all combined
   * robots together when at one robot encounters an error.
   */
  void triggerError();

  /**
   * Getter for the error flag of the class.
   *
   * @return True in case of an error false otherwise.
   */
  bool hasError() const noexcept;

  /**
   * Recovers the libfranka robot, resets the error flag and publishes the error state.
   */
  void resetError();

  /**
   * Returns whether the controller needs to be reset e.g. after error recovery.
   *
   * @return A copy of the controller_needs_reset flag.
   */
  bool controllerNeedsReset() const noexcept;

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
    {
      std::lock_guard<std::mutex> state_lock(libfranka_state_mutex_);
      robot_state_libfranka_ = robot_state;
    }

    std::lock_guard<std::mutex> command_lock(libfranka_cmd_mutex_);
    T current_cmd = command;
    if (has_error_ || !controller_active_) {
      return franka::MotionFinished(current_cmd);
    }
    return current_cmd;
  }

  void publishErrorState(bool error);

  void setupServicesAndActionServers(ros::NodeHandle& node_handle);

  void initRobot() override;

  bool setRunFunction(const ControlMode& requested_control_mode,
                      bool limit_rate,
                      double cutoff_frequency,
                      franka::ControllerMode internal_controller) override;

  void controlLoop();

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
