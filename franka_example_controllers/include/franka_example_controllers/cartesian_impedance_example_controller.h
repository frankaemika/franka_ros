#pragma once

#include <memory>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>

#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>

#include <eigen3/Eigen/Dense>
#include "pseudo_inversion.h"

#include <dynamic_reconfigure/server.h>
#include <franka_example_controllers/compliance_paramConfig.h>
#include <boost/scoped_ptr.hpp>

#include <geometry_msgs/PoseStamped.h>

namespace franka_example_controllers {

class CartesianImpedanceExampleController : public controller_interface::MultiInterfaceController<
                                            franka_hw::FrankaModelInterface,
                                            hardware_interface::EffortJointInterface,
                                            franka_hw::FrankaStateInterface> {
 public:
  CartesianImpedanceExampleController();
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle);
  void starting(const ros::Time&);
  void update(const ros::Time&, const ros::Duration& period);

 private:
  std::string arm_id_;
  ros::NodeHandle node_handle_;
  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  std::vector<hardware_interface::JointHandle> joint_handles_;
  std::vector<std::string> joint_names_;
  double pos_error_max_{0.3};
  double rot_error_max_{0.3};
  double k_p_nullspace_{100.0};
  double k_ext_{0.9};
  double filter_gain_{0.95};
  Eigen::Matrix<double, 6, 6> cartesian_stiffness_;
  Eigen::Matrix<double, 6, 6> cartesian_damping_;
  Eigen::Matrix<double, 6, 1> dx_;
  Eigen::Matrix<double, 7, 1> q_d_nullspace_;
  Eigen::Vector3d position_d_;
  Eigen::Quaterniond orientation_d_;

  // Dynamic reconfigure
  boost::scoped_ptr<dynamic_reconfigure::Server<franka_example_controllers::compliance_paramConfig>>
      dynamic_server_compliance_param_;
  ros::NodeHandle dynamic_reconfigure_compliance_param_node_;
  void compliance_param_callback(franka_example_controllers::compliance_paramConfig& config, uint32_t level);

  // Equilibrium pose subscriber
  ros::Subscriber sub_equilibrium_pose_;
  void equilibrium_pose_callback(const geometry_msgs::PoseStampedConstPtr &msg);
};

}  // namespace franka_example_controllers
