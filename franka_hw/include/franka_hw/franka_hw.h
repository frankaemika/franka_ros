#pragma once

#include <array>
#include <atomic>
#include <functional>
#include <string>
#include <vector>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <ros/node_handle.h>
#include <ros/time.h>

#include <franka/robot.h>

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
  * @param[in] arm_id Unique identifier for the Franka arm that the class
  * controls
  * @param[in] nh A nodehandle e.g to get parameters
  */
  FrankaHW(const std::vector<std::string>& joint_names,
           const std::string& arm_id,
           const ros::NodeHandle& node_handle);

  ~FrankaHW() override = default;

  /**
   * Runs the currently active controller in a realtime loop.
   *
   * If no controller is active, the function immediately exits.
   * When running a controller, the function only exits when ros_callback
   * returns false.
   *
   * @param[in] robot Robot instance.
   * @param[in] ros_callback A callback function that is executed at each time
   * step and runs all ROS-side functionality of the hardware. Execution is
   * stopped if it returns false.
   *
   * @throw franka::ControlException if an error related to control occurred.
   * @throw franka::NetworkException if the connection is lost, e.g. after a
   * timeout.
   * @throw franka::RealtimeException if realtime priority can not be set for
   * the current thread.
   */
  void control(
      franka::Robot& robot,
      std::function<bool(const ros::Time&, const ros::Duration&)> ros_callback);

  /**
   * Updates the controller interfaces from the given robot state.
   *
   * @param[in] robot_state Current robot state.
   * @param[in] reset If true, resets the controller interfaces.
   *
   * @throw franka::NetworkException if the connection is lost.
   */
  void update(const franka::RobotState& robot_state, bool reset = false);

  /**
   * Indicates whether there is an active controller.
   *
   * @return True if a controller is currently active, false otherwise.
   */
  bool controllerActive() const noexcept;

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
      const std::list<hardware_interface::ControllerInfo>& info) const override;

  /**
  * Performs the switch between controllers and is real-time capable
  *
  */
  void doSwitch(const std::list<hardware_interface::ControllerInfo>&,
                const std::list<hardware_interface::ControllerInfo>&) override;

  /**
  * Prepares the switching between controllers. This function is not real-time
  * capable.
  *
  * @param[in] start_list Information list about all controllers requested to be
  * started
  */
  bool prepareSwitch(
      const std::list<hardware_interface::ControllerInfo>& start_list,
      const std::list<hardware_interface::ControllerInfo>& stop_list) override;

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
    robot_state_ = robot_state;
    if (!controller_active_) {
      return franka::Stop;
    }
    if (ros_callback && !ros_callback()) {
      return franka::Stop;
    }
    return command;
  }

  hardware_interface::JointStateInterface joint_state_interface_{};
  franka_hw::FrankaStateInterface franka_state_interface_{};
  hardware_interface::PositionJointInterface position_joint_interface_{};
  hardware_interface::VelocityJointInterface velocity_joint_interface_{};
  hardware_interface::EffortJointInterface effort_joint_interface_{};
  franka_hw::FrankaPoseCartesianInterface franka_pose_cartesian_interface_{};
  franka_hw::FrankaVelocityCartesianInterface
      franka_velocity_cartesian_interface_{};

  joint_limits_interface::PositionJointSoftLimitsInterface
      position_joint_limit_interface_{};
  joint_limits_interface::VelocityJointSoftLimitsInterface
      velocity_joint_limit_interface_{};
  joint_limits_interface::EffortJointSoftLimitsInterface
      effort_joint_limit_interface_{};

  franka::RobotState robot_state_{};

  std::vector<std::string> joint_names_;
  const std::string arm_id_;

  franka::JointPositions position_joint_command_;
  franka::JointVelocities velocity_joint_command_;
  franka::Torques effort_joint_command_;
  franka::CartesianPose pose_cartesian_command_;
  franka::CartesianVelocities velocity_cartesian_command_;

  std::function<void(franka::Robot&, std::function<bool()>)> run_function_;

  std::atomic_bool controller_active_{false};
  ControlMode current_control_mode_ = ControlMode::None;
};

}  // namespace franka_hw
