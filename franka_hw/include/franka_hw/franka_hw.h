#pragma once

#include <array>
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
#include <franka_hw/franka_cartesian_state_interface.h>
#include <franka_hw/franka_controller_switching_types.h>
#include <franka_hw/franka_joint_command_interface.h>
#include <franka_hw/franka_joint_state_interface.h>
#include <franka_hw/trigger_rate.h>

namespace franka_hw {

class FrankaHW : public hardware_interface::RobotHW {
 public:
  FrankaHW() = delete;

  /**
  * Constructs an instance of FrankaHW
  *
  * @param[in] joint_names A vector of joint names for all franka joint
  * @param[in] robot A pointer to an istance of franka::Robot
  * @param[in] publish_rate Publish rate [Hz] for ROS topics
  * @param[in] arm_id Unique identifier for the Franka arm that the class
  * controls
  * @param[in] nh A nodehandle e.g to register publishers
  */
  FrankaHW(const std::vector<std::string>& joint_names,
           franka::Robot* robot,
           double publish_rate,
           const std::string& arm_id,
           const ros::NodeHandle& node_handle);

  ~FrankaHW() override = default;

  /**
  * Runs the control in case a valid run_function_ was chosen based on the
  * claimed resources
  *
  * @param[in] ros_callback A callback function that is executed at each time
  * step and runs all ros-side functionality of the hardware
  */
  void run(std::function<void(void)> ros_callback);

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

  /**
  * Callback function to send Joint Position commands
  *
  * @param[in] ros_callback A callback function that is executed at each time
  * step and runs all ros-side functionality of the hardware
  */
  void runJointPosition(std::function<void(void)> ros_callback);

  /**
  * Callback function to send Joint Velocity commands
  *
  * @param[in] ros_callback A callback function that is executed at each time
  * step and runs all ros-side functionality of the hardware
  */
  void runJointVelocity(std::function<void(void)> ros_callback);

  /**
  * Callback function to send Cartesian Pose commands
  *
  * @param[in] ros_callback A callback function that is executed at each time
  * step and runs all ros-side functionality of the hardware
  */
  void runCartesianPose(std::function<void(void)> ros_callback);

  /**
  * Callback function to send Cartesian Velocity commands
  *
  * @param[in] ros_callback A callback function that is executed at each time
  * step and runs all ros-side functionality of the hardware
  */
  void runCartesianVelocity(std::function<void(void)> ros_callback);

  /**
  * Callback function to send Joint Torque commands
  *
  * @param[in] ros_callback A callback function that is executed at each time
  * step and runs all ros-side functionality of the hardware
  */
  void runJointTorqueControl(std::function<void(void)> ros_callback);

  /**
  * Callback function to send Joint Torque commands and use the Joint Position
  * motion generator of the robot
  *
  * @param[in] ros_callback A callback function that is executed at each time
  * step and runs all ros-side functionality of the hardware
  */
  void runTorqueControlWithJointPositionMotionGenerator(
      std::function<void(void)> ros_callback);

  /**
  * Callback function to send Joint Torque commands and use the Joint Velocity
  * motion generator of the robot
  *
  * @param[in] ros_callback A callback function that is executed at each time
  * step and runs all ros-side functionality of the hardware
  */
  void runTorqueControlWithJointVelocityMotionGenerator(
      std::function<void(void)> ros_callback);

  /**
  * Callback function to send Joint Torque commands and use the Cartesian pose
  * motion generator of the robot
  *
  * @param[in] ros_callback A callback function that is executed at each time
  * step and runs all ros-side functionality of the hardware
  */
  void runTorqueControlWithCartesianPoseMotionGenerator(
      std::function<void(void)> ros_callback);

  /**
  * Callback function to send Joint Torque commands and use the Cartesian
  * velocity motion generator of the robot
  *
  * @param[in] ros_callback A callback function that is executed at each time
  * step and runs all ros-side functionality of the hardware
  */
  void runTorqueControlWithCartesianVelocityMotionGenerator(
      std::function<void(void)> ros_callback);

 private:
  hardware_interface::JointStateInterface joint_state_interface_;
  franka_hw::FrankaJointStateInterface franka_joint_state_interface_;
  franka_hw::FrankaCartesianStateInterface franka_cartesian_state_interface_;
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

  franka::Robot* robot_;

  franka_hw::TriggerRate publish_rate_;
  realtime_tools::RealtimePublisher<tf2_msgs::TFMessage> publisher_transforms_;
  realtime_tools::RealtimePublisher<franka_hw::FrankaState>
      publisher_franka_states_;
  realtime_tools::RealtimePublisher<sensor_msgs::JointState>
      publisher_joint_states_;
  realtime_tools::RealtimePublisher<geometry_msgs::WrenchStamped>
      publisher_external_wrench_;

  std::vector<std::string> joint_names_;
  std::string arm_id_;
  franka::RobotState robot_state_;

  std::array<double, 7> position_joint_command_ = {
      {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  std::array<double, 7> velocity_joint_command_ = {
      {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  std::array<double, 7> effort_joint_command_ = {
      {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  std::array<double, 16> pose_cartesian_command_ = {
      {1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
       0.0, 1.0}};
  std::array<double, 6> velocity_cartesian_command_ = {
      {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  uint64_t sequence_number_joint_states_ = 0;
  uint64_t sequence_number_franka_states_ = 0;
  bool controller_running_flag_ = false;
  std::function<void(std::function<void(void)>)> run_function_;
};

}  // namespace franka_hw
