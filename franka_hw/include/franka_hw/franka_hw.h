// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <array>
#include <atomic>
#include <exception>
#include <functional>
#include <list>
#include <string>

#include <franka/control_types.h>
#include <franka/duration.h>
#include <franka/rate_limiting.h>
#include <franka/robot.h>
#include <franka/robot_state.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <ros/time.h>
#include <urdf/model.h>

#include <franka_hw/control_mode.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <franka_hw/model_base.h>
#include <franka_hw/resource_helpers.h>

namespace franka_hw {

/**
 * This class wraps the functionality of libfranka for controlling Panda robots into the ros_control
 * framework.
 */
class FrankaHW : public hardware_interface::RobotHW {
 public:
  /**
   * Default constructor.
   * Note: Be sure to call the init() method before operation.
   */
  FrankaHW();

  virtual ~FrankaHW() override = default;

  /**
   * Initializes the FrankaHW class to be fully operational. This involves parsing required
   * configurations from the ROS parameter server, connecting to the robot and setting up interfaces
   * for the ros_control framework.
   *
   * @param[in] root_nh A node handle in the root namespace of the control node.
   * @param[in] robot_hw_nh A node handle in the namespace of the robot hardware.
   *
   * @return True if successful, false otherwise.
   */
  virtual bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) override;

  /**
   * Reads the parameterization of the hardware class from the ROS parameter server
   * (e.g. arm_id, robot_ip joint_names etc.)
   *
   * @param[in] root_nh A node handle in the root namespace of the control node.
   * @param[in] robot_hw_nh A node handle in the namespace of the robot hardware.
   *
   * @return True if successful, false otherwise.
   */
  virtual bool initParameters(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh);

  /**
   * Initializes the class in terms of ros_control interfaces.
   * Note: You have to call initParameters beforehand. Use the complete initialization routine
   * \ref init() method to control robots.
   *
   * @param[in] robot_hw_nh A node handle in the namespace of the robot hardware.
   * @return True if successful, false otherwise.
   */
  virtual void initROSInterfaces(ros::NodeHandle& robot_hw_nh);

  /**
   * Initializes the callbacks for on-the-fly reading the parameters for rate limiting,
   * internal controllers and cutoff frequency.
   *
   * @param[in] robot_hw_nh A node handle in the namespace of the robot hardware.
   */
  virtual void setupParameterCallbacks(ros::NodeHandle& robot_hw_nh);

  /**
   * Create a libfranka robot, connecting the hardware class to the master controller.
   * Note: While the robot is connected, no DESK based tasks can be executed.
   */
  virtual void connect();

  /**
   * Tries to disconnect the hardware class from the robot, freeing it for e.g. DESK-based tasks.
   * Note: Disconnecting is only possible when no controller is actively running.
   * @return true if successfully disconnected, false otherwise.
   */
  virtual bool disconnect();

  /**
   * Checks whether the hardware class is connected to a robot.
   * @return true if connected, false otherwise.
   */
  virtual bool connected();

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
  virtual void doSwitch(
      const std::list<hardware_interface::ControllerInfo>& /*start_list*/,
      const std::list<hardware_interface::ControllerInfo>& /*stop_list*/) override;

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
   * @return The current joint position command.
   */
  virtual std::array<double, 7> getJointPositionCommand() const noexcept;

  /**
   * Gets the current joint velocity command.
   *
   * @return The current joint velocity command.
   */
  virtual std::array<double, 7> getJointVelocityCommand() const noexcept;

  /**
   * Gets the current joint torque command.
   *
   * @return The current joint torque command.
   */
  virtual std::array<double, 7> getJointEffortCommand() const noexcept;

  /**
   * Enforces limits on position, velocity, and torque level.
   *
   * @param[in] period The duration of the current cycle.
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
  virtual void read(const ros::Time& time, const ros::Duration& period) override;

  /**
   * Writes data to the franka robot.
   *
   * @param[in] time The current time.
   * @param[in] period The time passed since the last call to \ref write.
   */
  virtual void write(const ros::Time& time, const ros::Duration& period) override;

  /**
   * Getter for the libfranka robot instance.
   * @throw std::logic_error in case the robot is not connected or the class in not initialized
   */
  virtual franka::Robot& robot() const;

  /**
   * Getter for the mutex protecting access to the libfranka::robot class. This enables thread-safe
   * access to robot().
   * @return A reference to the robot mutex.
   */
  virtual std::mutex& robotMutex();

  /**
   * Checks a command for NaN values.
   *
   * @param[in] command  The command to check.
   *
   * @return True if the command contains NaN, false otherwise.
   */
  static bool commandHasNaN(const franka::Torques& command);

  /**
   * Checks a command for NaN values.
   *
   * @param[in] command The command to check.
   *
   * @return True if the command contains NaN, false otherwise.
   */
  static bool commandHasNaN(const franka::JointPositions& command);

  /**
   * Checks a command for NaN values.
   *
   * @param[in] command The command to check.
   *
   * @return True if the command contains NaN, false otherwise.
   */
  static bool commandHasNaN(const franka::JointVelocities& command);

  /**
   * Checks a command for NaN values.
   *
   * @param[in] command  The command to check.
   *
   * @return True if the command contains NaN, false otherwise.
   */
  static bool commandHasNaN(const franka::CartesianPose& command);

  /**
   * Checks a command for NaN values.
   *
   * @param[in] command  The command to check.
   *
   * @return True if the command contains NaN, false otherwise.
   */
  static bool commandHasNaN(const franka::CartesianVelocities& command);

  /**
   * Parses a set of collision thresholds from the parameter server. The methods returns
   * the default values if no parameter was found or the size of the array did not match
   * the defaults dimension.
   *
   * @param[in] name The name of the parameter to look for.
   * @param[in] robot_hw_nh A node handle in the namespace of the robot hardware.
   * @param[in] defaults A set of default values that also specify the size the parameter must have
   * to be valid.
   * @return A set parsed parameters if valid parameters where found, the default values otherwise.
   */
  static std::vector<double> getCollisionThresholds(const std::string& name,
                                                    const ros::NodeHandle& robot_hw_nh,
                                                    const std::vector<double>& defaults);

 protected:
  /**
   * Checks whether an array of doubles contains NaN values.
   *
   * @param[in] command  array The array to check.
   *
   * @return True if the array contains NaN values, false otherwise.
   */
  template <size_t size>
  static bool arrayHasNaN(const std::array<double, size>& array) {
    return std::any_of(array.begin(), array.end(), [](const double& e) { return std::isnan(e); });
  }

  using Callback = std::function<bool(const franka::RobotState&, franka::Duration)>;

  /**
   * Callback for the libfranka control loop. This method is designed to incorporate a
   * second callback named ros_callback that will be called on each iteration of the callback.
   *
   * @param[in] command The datafield containing the command to send to the robot.
   * @param[in] ros_callback An additional callback that is executed every time step.
   * @param[in] robot_state The current robot state to compute commands with.
   * @param[in] time_step Time since last call to the callback.
   * @throw std::invalid_argument When a command contains NaN values.
   *
   * @return The command to be sent to the robot via libfranka.
   */
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

  /**
   * Configures a limit interface to enforce limits on effort, velocity or position level
   * on joint commands.
   *
   * @param[out] limit_interface The limit interface to set up.
   * @param[out] command_interface The command interface to hook the limit interface to.
   */
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

  /**
   * Configures and registers the joint state interface in ros_control.
   *
   * @param[in] robot_state The data field holding the updated robot state.
   */
  virtual void setupJointStateInterface(franka::RobotState& robot_state);

  /**
   * Configures and registers a joint command interface for positions velocities or efforts in
   * ros_control.
   *
   * @param[in] command The data field holding the command to execute.
   * @param[in] state The data field holding the updated robot state.
   * @param[in] use_q_d Flag to configure using desired values as joint_states.
   * @param[out] interface The command interface to configure.
   */
  template <typename T>
  void setupJointCommandInterface(std::array<double, 7>& command,
                                  franka::RobotState& state,
                                  bool use_q_d,
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

  /**
   * Configures and registers the state interface offering the full franka::RobotState in
   * ros_control.
   *
   * @param[in] robot_state The data field holding the updated robot state.
   */
  virtual void setupFrankaStateInterface(franka::RobotState& robot_state);

  /**
   * Configures and registers the command interface for Cartesian poses in ros_control.
   *
   * @param[in] pose_cartesian_command The data field holding the command to execute.
   */
  virtual void setupFrankaCartesianPoseInterface(franka::CartesianPose& pose_cartesian_command);

  /**
   * Configures and registers the command interface for Cartesian velocities in ros_control.
   *
   * @param[in] velocity_cartesian_command The data field holding the command to execute.
   */
  virtual void setupFrankaCartesianVelocityInterface(
      franka::CartesianVelocities& velocity_cartesian_command);

  /**
   * Configures and registers the model interface offering kinematics and dynamics in ros_control.
   *
   * @param[in] robot_state A reference to the data field storing the current robot state. This
   * state is used to evaluate model qunatities (by default) at the current state.
   */
  virtual void setupFrankaModelInterface(franka::RobotState& robot_state);

  /**
   * Configures the run function which is used as libfranka control callback based on the requested
   * control mode.
   *
   * @param[in] requested_control_mode The control mode to configure (e.g. torque/position/velocity
   * etc.)
   * @param[in] limit_rate Flag to enable/disable rate limiting to smoothen the commands.
   * @param[in] cutoff_frequency The cutoff frequency applied for command smoothing.
   * @param[in] internal_controller The internal controller to use when using position or velocity
   * modes.
   *
   * @return True if successful, false otherwise.
   */
  virtual bool setRunFunction(const ControlMode& requested_control_mode,
                              bool limit_rate,
                              double cutoff_frequency,
                              franka::ControllerMode internal_controller);
  /**
   * Uses the robot_ip_ to connect to the robot via libfranka and loads the libfranka model.
   */
  virtual void initRobot();

  struct CollisionConfig {
    std::array<double, 7> lower_torque_thresholds_acceleration;
    std::array<double, 7> upper_torque_thresholds_acceleration;
    std::array<double, 7> lower_torque_thresholds_nominal;
    std::array<double, 7> upper_torque_thresholds_nominal;
    std::array<double, 6> lower_force_thresholds_acceleration;
    std::array<double, 6> upper_force_thresholds_acceleration;
    std::array<double, 6> lower_force_thresholds_nominal;
    std::array<double, 6> upper_force_thresholds_nominal;
  };

  CollisionConfig collision_config_;
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

  std::mutex robot_mutex_;
  std::unique_ptr<franka::Robot> robot_;
  std::unique_ptr<franka_hw::ModelBase> model_;

  std::array<std::string, 7> joint_names_;
  std::string arm_id_;
  std::string robot_ip_;
  urdf::Model urdf_model_;
  double joint_limit_warning_threshold_{0.1};
  franka::RealtimeConfig realtime_config_;

  bool initialized_{false};
  std::atomic_bool controller_active_{false};
  ControlMode current_control_mode_ = ControlMode::None;

  std::function<franka::ControllerMode()> get_internal_controller_;
  std::function<bool()> get_limit_rate_;
  std::function<double()> get_cutoff_frequency_;
  std::function<void(franka::Robot&, Callback)> run_function_;
};

}  // namespace franka_hw
