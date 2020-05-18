// Copyright (c) 2020 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <franka_example_controllers/teleop_paramConfig.h>
#include <franka_hw/franka_state_interface.h>
#include <franka_hw/trigger_rate.h>

#include <control_msgs/GripperCommandAction.h>
#include <controller_interface/multi_interface_controller.h>
#include <dynamic_reconfigure/server.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/node_handle.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>

#include <Eigen/Dense>
#include <mutex>

namespace franka_example_controllers {

/**
 * Controller class for ros_control that tracks joint positions and joint velocities of a master
 * arm and applies them to a slave arm. It also applies force-feedback to the master arm.
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
  void starting(const ros::Time& time) override;

  /**
   * Computes the control-law and commands the resulting joint torques to the robots.
   *
   * @param[in] period The control period (here 0.001s)
   */
  void update(const ros::Time& time, const ros::Duration& period) override;

 private:
  using Vector6d = Eigen::Matrix<double, 6, 1>;
  using Vector7d = Eigen::Matrix<double, 7, 1>;
  using Matrix7d = Eigen::Matrix<double, 7, 7>;

  struct FrankaDataContainer {
    std::unique_ptr<franka_hw::FrankaStateHandle> state_handle;
    std::vector<hardware_interface::JointHandle> joint_handles;

    Vector7d tau_target;
    Vector7d tau_target_last;
    Vector7d q;
    Vector7d dq;

    double f_ext_norm;
    double contact;
    double contact_ramp_increase;
    double contact_force_threshold;
  };

  FrankaDataContainer master_data_;
  FrankaDataContainer slave_data_;

  // positions and velocities for slave arm
  Vector7d q_target_;
  Vector7d q_target_last_;
  Vector7d dq_calc_;
  Vector7d dq_target_;
  Vector7d dq_target_last_;

  Vector7d dq_max_lower_;
  Vector7d dq_max_upper_;
  Vector7d ddq_max_lower_;
  Vector7d ddq_max_upper_;
  double velocity_ramp_shift_;     // parameter for ramping dq_max and ddq_max in rad
  double velocity_ramp_increase_;  // parameter for ramping dq_max and ddq_max

  Vector7d k_p_slave_;   // p-gain for slave arm
  Vector7d k_d_slave_;   // d-gain for slave arm
  Vector7d k_d_master_;  // d-gain for master arm
  Vector7d k_dq_;        // gain for drift compensation in slave arm

  double force_feedback_idle_;     // Applied force-feedback, when master arm is not guided
  double force_feedback_guiding_;  // Applied force-feeback, when master arm is guided

  bool initArm(hardware_interface::RobotHW* robot_hw,
               FrankaDataContainer& arm_data,
               const std::string& arm_id,
               const std::vector<std::string>& joint_names);

  void updateArm(FrankaDataContainer& arm_data);

  Vector7d saturateAndLimit(const Vector7d& x_calc,
                            const Vector7d& x_last,
                            const Vector7d& x_max,
                            const Vector7d& dx_max,
                            const double& delta_t);

  double rampParameter(const double& x,
                       const double& left_bound,
                       const double& right_bound,
                       const double& shift_along_x,
                       const double& increase_factor);

  // Debug tool
  bool debug_;
  std::mutex dynamic_reconfigure_mutex_;
  double master_damping_factor_{1.0};
  double slave_p_gain_factor_{1.0};
  double slave_d_gain_factor_{1.0};

  ros::NodeHandle dynamic_reconfigure_teleop_param_node_;
  std::unique_ptr<dynamic_reconfigure::Server<franka_example_controllers::teleop_paramConfig>>
      dynamic_server_teleop_param_;
  void teleopParamCallback(franka_example_controllers::teleop_paramConfig& config, uint32_t level);

  franka_hw::TriggerRate publish_rate_;
  realtime_tools::RealtimePublisher<sensor_msgs::JointState> master_target_pub_;
  realtime_tools::RealtimePublisher<sensor_msgs::JointState> slave_target_pub_;
  realtime_tools::RealtimePublisher<std_msgs::Float64> master_contact_pub_;
  realtime_tools::RealtimePublisher<std_msgs::Float64> slave_contact_pub_;

  void publishMasterTarget();
  void publishSlaveTarget();
  void publishMasterContact();
  void publishSlaveContact();
};
}  // namespace franka_example_controllers
