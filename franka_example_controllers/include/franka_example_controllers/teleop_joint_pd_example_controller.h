// Copyright (c) 2020 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <franka_example_controllers/joint_wall.h>
#include <franka_example_controllers/teleop_paramConfig.h>
#include <franka_hw/franka_state_interface.h>
#include <franka_hw/trigger_rate.h>

#include <control_msgs/GripperCommandAction.h>
#include <controller_interface/multi_interface_controller.h>
#include <dynamic_reconfigure/server.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Dense>
#include <memory>
#include <mutex>
#include <vector>

namespace franka_example_controllers {
/**
 * Finite state machine that defines the states of the teleoperation phases.
 * ALIGN is the initial phase, when the leader and follower align. During this phase the leader
 * cannot be moved.
 * TRACK is the tracking phase.
 */
enum TeleopStateMachine {
  ALIGN,
  TRACK,
};

/**
 * Controller class for ros_control that allows force-feedback teleoperation of a follower arm from
 * a leader arm. Smooth tracking is implemented by integrating a velocity signal, which is
 * calculated by limiting and saturating the velocity of the leader arm and a drift compensation.
 * The torque control of the follower arm is implemented by a simple PD-controller.
 * The leader arm is slightly damped to reduce vibrations.
 * Force-feedback is applied to the leader arm when the external forces on the follower arm exceed a
 * configured threshold.
 * While the leader arm is unguided (not in contact), the applied force-feedback will be reduced.
 */
class TeleopJointPDExampleController : public controller_interface::MultiInterfaceController<
                                           hardware_interface::EffortJointInterface,
                                           franka_hw::FrankaStateInterface> {
 public:
  /**
   * Initializes the controller class to be ready to run.
   *
   * @param[in] robot_hw Pointer to a RobotHW class to get interfaces and resource handles.
   * @param[in] node_handle Nodehandle that allows getting parameterizations from the server and
   * starting publisher
   */
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;

  /**
   * Prepares the controller for the real-time execution. This method is executed once every time
   * the controller is started and runs in real-time
   */
  void starting(const ros::Time& /*time*/) override;

  /**
   * Computes the control-law and commands the resulting joint torques to the robots.
   *
   * @param[in] period The control period (here 0.001s)
   */
  void update(const ros::Time& /*time*/, const ros::Duration& period) override;

 private:
  using Vector6d = Eigen::Matrix<double, 6, 1>;
  using Vector7d = Eigen::Matrix<double, 7, 1>;
  using Matrix7d = Eigen::Matrix<double, 7, 7>;

  struct FrankaDataContainer {
    std::unique_ptr<franka_hw::FrankaStateHandle> state_handle;
    std::vector<hardware_interface::JointHandle> joint_handles;

    // A virtual wall to avoid joint limits.
    std::unique_ptr<JointWallContainer<7>> virtual_joint_wall;

    Vector7d tau_target;       // Target effort of each joint [Nm, Nm, Nm, Nm, Nm, Nm, Nm]
    Vector7d tau_target_last;  // Last target effort of each joint [Nm, ...]
    Vector7d q;                // Measured position of each joint [rad, ...]
    Vector7d dq;               // Measured velocity of each joint [rad/s, ...]

    double f_ext_norm;  // Norm of the external (cartesian) forces vector at the EE [N]
    double contact;     // Contact scaling factor (values between 0 and 1)
    double contact_ramp_increase{0.3};  // Parameter for contact scaling factor
    double contact_force_threshold;     // Parameter for contact scaling factor [N]
  };

  FrankaDataContainer leader_data_;    // Container for data of the leader arm
  FrankaDataContainer follower_data_;  // Container for data of the follower arm

  Vector7d q_target_;  // Target positions of the follower arm [rad, rad, rad, rad, rad, rad, rad]
  Vector7d q_target_last_;   // Last target positions of the follower arm [rad, ...]
  Vector7d dq_unsaturated_;  // Unsaturated target velocities of the follower arm [rad/s, ...]
  Vector7d dq_target_;       // Target velocities of the follower arm [rad/s, ...]
  Vector7d dq_target_last_;  // Last target velocities of the follower arm [rad/s, ...]

  Vector7d init_leader_q_;  // Measured q of leader arm during alignment [rad]

  Vector7d dq_max_lower_;              // Lower max velocities of the follower arm [rad/s, ...]
  Vector7d dq_max_upper_;              // Upper max velocities of the follower arm [rad/s, ...]
  Vector7d ddq_max_lower_;             // Lower max accelerations of the follower arm [rad/s², ...]
  Vector7d ddq_max_upper_;             // Upper max accelerations of the follower arm [rad/s², ...]
  double velocity_ramp_shift_{0.25};   // parameter for ramping dq_max and ddq_max [rad]
  double velocity_ramp_increase_{20};  // parameter for ramping dq_max and ddq_max

  // Max velocities of the follower arm during alignment [rad/s, ...]
  Vector7d dq_max_align_{(Vector7d() << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1).finished()};
  // Max accelerations of the follower arm during alignment [rad/s², ...]
  Vector7d ddq_max_align_{(Vector7d() << 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5).finished()};

  Vector7d alignment_error_;  // Diff between follower and leader q during alignment [rad/s, ...]
  Vector7d prev_alignment_error_;  // alignment_error_ in previous control loop [rad/s, ...]

  Vector7d k_p_follower_;  // p-gain for follower arm
  Vector7d k_d_follower_;  // d-gain for follower arm
  // p-gain for follower arm during alignment
  Vector7d k_p_follower_align_{(Vector7d() << 45.0, 45.0, 45.0, 45.0, 18.0, 11.0, 5.0).finished()};
  // d-gain for follower arm during alignment
  Vector7d k_d_follower_align_{(Vector7d() << 4.5, 4.5, 4.5, 4.5, 1.5, 1.5, 1.0).finished()};

  Vector7d dq_max_leader_lower_;  // Soft max velocities of the leader arm [rad/s, ...]
  Vector7d dq_max_leader_upper_;  // Hard max velocities of the leader arm [rad/s, ...]
  Vector7d k_d_leader_lower_;     // d-gain for leader arm when under soft-limit
  Vector7d k_d_leader_upper_;     // d-gain for leader arm when hard limit is reached

  Vector7d k_dq_;  // gain for drift compensation in follower arm

  double force_feedback_idle_{0.5};      // Applied force-feedback, when leader arm is not guided
  double force_feedback_guiding_{0.95};  // Applied force-feeback, when leader arm is guided

  double decrease_factor_{0.95};  // Param, used when (in error state) controlling torques to zero

  TeleopStateMachine current_state_{TeleopStateMachine::ALIGN};  // Current state in teleoperation

  const double kAlignmentTolerance_{1e-2};  // Tolerance to consider a joint aligned [rad]

  void initArm(hardware_interface::RobotHW* robot_hw,
               ros::NodeHandle& node_handle,
               FrankaDataContainer& arm_data,
               const std::string& arm_id,
               const std::vector<std::string>& joint_names);

  void updateArm(FrankaDataContainer& arm_data);

  Vector7d saturateAndLimit(const Vector7d& x_calc,
                            const Vector7d& x_last,
                            const Vector7d& x_max,
                            const Vector7d& dx_max,
                            double delta_t);

  double rampParameter(double x,
                       double neg_x_asymptote,
                       double pos_x_asymptote,
                       double shift_along_x,
                       double increase_factor);

  template <typename T>
  std::vector<T> getJointParams(const std::string& param_name, ros::NodeHandle& nh) {
    std::vector<T> vec;
    if (!nh.getParam(param_name, vec) || vec.size() != 7) {
      throw std::invalid_argument("TeleopJointPDExampleController: Invalid or no parameter " +
                                  nh.getNamespace() + "/" + param_name +
                                  " provided, aborting controller init!");
    }
    return vec;
  }

  Vector7d get7dParam(const std::string& param_name, ros::NodeHandle& nh);

  template <typename T>
  T get1dParam(const std::string& param_name, ros::NodeHandle& nh) {
    T out;
    if (!nh.getParam(param_name, out)) {
      throw std::invalid_argument("TeleopJointPDExampleController: Invalid or no parameter " +
                                  nh.getNamespace() + "/" + param_name +
                                  " provided, "
                                  "aborting controller init!");
    }
    return out;
  }

  static void getJointLimits(ros::NodeHandle& nh,
                             const std::vector<std::string>& joint_names,
                             std::array<double, 7>& upper_joint_soft_limit,
                             std::array<double, 7>& lower_joint_soft_limit);

  Vector7d leaderDamping(const Vector7d& dq);

  // Debug tool
  bool debug_{false};
  std::mutex dynamic_reconfigure_mutex_;
  double leader_damping_scaling_{1.0};
  double follower_stiffness_scaling_{1.0};

  ros::NodeHandle dynamic_reconfigure_teleop_param_node_;
  std::unique_ptr<dynamic_reconfigure::Server<franka_example_controllers::teleop_paramConfig>>
      dynamic_server_teleop_param_;
  void teleopParamCallback(franka_example_controllers::teleop_paramConfig& config, uint32_t level);

  franka_hw::TriggerRate publish_rate_{60.0};
  realtime_tools::RealtimePublisher<sensor_msgs::JointState> leader_target_pub_;
  realtime_tools::RealtimePublisher<sensor_msgs::JointState> follower_target_pub_;
  realtime_tools::RealtimePublisher<std_msgs::Float64> leader_contact_pub_;
  realtime_tools::RealtimePublisher<std_msgs::Float64> follower_contact_pub_;
  realtime_tools::RealtimePublisher<visualization_msgs::MarkerArray> marker_pub_;

  void publishLeaderTarget();
  void publishFollowerTarget();
  void publishLeaderContact();
  void publishFollowerContact();
  void publishMarkers();
};
}  // namespace franka_example_controllers
