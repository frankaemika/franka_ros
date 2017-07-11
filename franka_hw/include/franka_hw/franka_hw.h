#pragma once

#include <array>
#include <cinttypes>
#include <functional>
#include <string>
#include <vector>

#include <geometry_msgs/WrenchStamped.h>
#include <hardware_interface/controller_info.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <realtime_tools/realtime_publisher.h>
#include <sensor_msgs/JointState.h>
#include <tf2_msgs/TFMessage.h>

#include <franka/robot.h>

#include <franka_hw/FrankaState.h>

#include <franka_hw/franka_cartesian_command_interface.h>
#include <franka_hw/franka_controller_switching_types.h>
#include <franka_hw/franka_state_interface.h>

namespace franka_hw {

class FrankaHW : public hardware_interface::RobotHW {
 public:
  static constexpr double kMaximumJointAcceleration{1.0};
  static constexpr double kMaximumJointJerk{4000.0};

  FrankaHW() = delete;

  /**
  * Constructs an instance of FrankaHW
  *
  * @param[in] joint_names A vector of joint names for all franka joint
  * @param[in] robot A pointer to an istance of franka::Robot
  * @param[in] arm_id Unique identifier for the Franka arm that the class
  * controls
  * @param[in] nh A nodehandle e.g to register publishers
  */
  FrankaHW(const std::vector<std::string>& joint_names,
           franka::Robot* robot,
           const std::string& arm_id,
           const ros::NodeHandle& node_handle);

  ~FrankaHW() override = default;

  /**
   * Runs the currently active controller in a realtime loop.
   *
   * If no controller is active, a default loop is run that reads the robot
   * state, but does not perform control. The function only exits if
   * ros_callback returns false.
   *
   * @param[in] ros_callback A callback function that is executed at each time
   * step and runs all ros-side functionality of the hardware. Execution is
   * stopped if it returns false.
   */
  void run(std::function<bool()> ros_callback);

  /**
  * Checks whether a requested controller can be run, based on the resources and
  * interfaces it claims
  *
  * @param[in] info A list of all controllers to be started, including the
  * resources they claim
  * @return Returns true in case of a conflict, false in case of valid
  * controllers
  */
  bool checkForConflict(
      const std::list<hardware_interface::ControllerInfo>& info) const;

  /**
  * Performs the switch between controllers and is real-time capable
  *
  * @param[in] start_list Information list about all controllers requested to be
  * started
  * @param[in] stop_list Information list about all controllers requested to be
  * stopped
  */
  void doSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                const std::list<hardware_interface::ControllerInfo>& stop_list);

  /**
  * Prepares the switching between controllers. This function is not real-time
  * capable.
  *
  * @param[in] start_list Information list about all controllers requested to be
  * started
  * @param[in] stop_list Information list about all controllers requested to be
  * stopped
  */
  bool prepareSwitch(
      const std::list<hardware_interface::ControllerInfo>& start_list,
      const std::list<hardware_interface::ControllerInfo>& stop_list);

  /**
  * Publishes all relevant data received from the Franka arm
  */
  void publishFrankaStates();

  /**
  * Publishes the joint states of the Franka arm
  */
  void publishJointStates();

  /**
  * Publishes the transforms for EE and K frame which define the end-effector
  * (EE) and the Cartesian impedance reference frame (K)
  */
  void publishTransforms();

  /**
  * Publishes the estimated external wrench felt by the Franka
  */
  void publishExternalWrench();

  /**
  * Getter for the current Joint Position Command
  *
  * @return The current Joint Position command
  */
  std::array<double, 7> getJointPositionCommand() const;

  /**
  * Getter for the current Joint Velocity Command
  *
  * @return The current Joint Velocity command
  */
  std::array<double, 7> getJointVelocityCommand() const;

  /**
  * Getter for the current Joint Torque Command
  *
  * @return The current Joint Torque command
  */
  std::array<double, 7> getJointEffortCommand() const;

  /**
  * Enforces joint limits on position velocity and torque level
  *
  * @param[in] kPeriod The duration of the current cycle
  */
  void enforceLimits(const ros::Duration kPeriod);

 private:
  template <typename T>
  T controlCallback(const T& command,
                    std::function<bool()> ros_callback,
                    const franka::RobotState& robot_state) {
    if (readCallback(ros_callback, robot_state)) {
      return command;
    }
    return franka::Stop;
  }

  bool readCallback(std::function<bool()> ros_callback,
                    const franka::RobotState& robot_state);

  hardware_interface::JointStateInterface joint_state_interface_;
  franka_hw::FrankaStateInterface franka_state_interface_;
  hardware_interface::PositionJointInterface position_joint_interface_;
  hardware_interface::VelocityJointInterface velocity_joint_interface_;
  hardware_interface::EffortJointInterface effort_joint_interface_;
  franka_hw::FrankaPoseCartesianInterface franka_pose_cartesian_interface_;
  franka_hw::FrankaVelocityCartesianInterface
      franka_velocity_cartesian_interface_;

  joint_limits_interface::PositionJointSoftLimitsInterface
      position_joint_limit_interface_;
  joint_limits_interface::VelocityJointSoftLimitsInterface
      velocity_joint_limit_interface_;
  joint_limits_interface::EffortJointSoftLimitsInterface
      effort_joint_limit_interface_;

  realtime_tools::RealtimePublisher<tf2_msgs::TFMessage> publisher_transforms_;
  realtime_tools::RealtimePublisher<franka_hw::FrankaState>
      publisher_franka_states_;
  realtime_tools::RealtimePublisher<sensor_msgs::JointState>
      publisher_joint_states_;
  realtime_tools::RealtimePublisher<geometry_msgs::WrenchStamped>
      publisher_external_wrench_;
  std::vector<std::string> joint_names_;
  const std::string arm_id_;

  franka::Robot* const robot_;
  franka::RobotState robot_state_;
  franka::JointPositions position_joint_command_;
  franka::JointVelocities velocity_joint_command_;
  franka::Torques effort_joint_command_;
  franka::CartesianPose pose_cartesian_command_;
  franka::CartesianVelocities velocity_cartesian_command_;

  uint64_t sequence_number_joint_states_ = 0;
  uint64_t sequence_number_franka_states_ = 0;
  bool controller_running_ = true;
  ControlMode current_control_mode_ = ControlMode::None;
  const std::function<void(std::function<bool()>)> default_run_function_;
  std::function<void(std::function<bool()>)> run_function_;
};

}  // namespace franka_hw
