// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <memory>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseStamped.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <Eigen/Dense>

#include <franka_combined_example_controllers/dual_arm_compliance_paramConfig.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <geometry_msgs/PoseStamped.h>

namespace franka_combined_example_controllers {

struct FrankaDataContainer {
  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  std::vector<hardware_interface::JointHandle> joint_handles_;

  double filter_params_{0.005};
  double nullspace_stiffness_{20.0};
  double nullspace_stiffness_target_{20.0};
  const double delta_tau_max_{1.0};
  Eigen::Matrix<double, 6, 6> cartesian_stiffness_;
  Eigen::Matrix<double, 6, 6> cartesian_stiffness_target_;
  Eigen::Matrix<double, 6, 6> cartesian_damping_;
  Eigen::Matrix<double, 6, 6> cartesian_damping_target_;
  Eigen::Matrix<double, 7, 1> q_d_nullspace_;
  Eigen::Vector3d position_d_;
  Eigen::Quaterniond orientation_d_;
  Eigen::Vector3d position_d_target_;
  Eigen::Quaterniond orientation_d_target_;
};

class DualArmCartesianImpedanceExampleController
    : public controller_interface::MultiInterfaceController<
          franka_hw::FrankaModelInterface,
          hardware_interface::EffortJointInterface,
          franka_hw::FrankaStateInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;

 private:
  std::map<std::string, FrankaDataContainer> arms_data_;
  std::string left_arm_id_;
  std::string right_arm_id_;

  // Saturation
  Eigen::Matrix<double, 7, 1> saturateTorqueRate(
      const FrankaDataContainer& arm_data,
      const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
      const Eigen::Matrix<double, 7, 1>& tau_J_d);  // NOLINT (readability-identifier-naming)

  // Init, update, starting function for a single arm
  bool initArm(hardware_interface::RobotHW* robot_hw,
               std::string& arm_id,
               std::vector<std::string>& joint_names);
  void updateArm(FrankaDataContainer& arm_data);
  void startingArm(FrankaDataContainer& arm_data);

  // Dynamic reconfigure
  std::unique_ptr<dynamic_reconfigure::Server<
      franka_combined_example_controllers::dual_arm_compliance_paramConfig>>
      dynamic_server_compliance_param_;
  ros::NodeHandle dynamic_reconfigure_compliance_param_node_;
  void complianceParamCallback(
      franka_combined_example_controllers::dual_arm_compliance_paramConfig& config,
      uint32_t level);

  // Target pose subscriber
  ros::Subscriber sub_target_pose_left_;
  ros::Subscriber sub_target_pose_right_;
  void targetPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg,
                          const std::string& arm_id);
};

}  // namespace franka_combined_example_controllers
