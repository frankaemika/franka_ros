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

#include <eigen3/Eigen/Core>

#include <dynamic_reconfigure/server.h>
#include <franka_example_controllers/desired_mass_paramConfig.h>
#include <boost/scoped_ptr.hpp>

namespace franka_example_controllers {

class ForceExampleController : public controller_interface::MultiInterfaceController<
                                   franka_hw::FrankaModelInterface,
                                   hardware_interface::EffortJointInterface,
                                   franka_hw::FrankaStateInterface> {
 public:
  ForceExampleController();
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle);
  void starting(const ros::Time&);
  void update(const ros::Time&, const ros::Duration& period);

 private:
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::vector<hardware_interface::JointHandle> joint_handles_;
  double desired_mass_{0.0};
  double target_mass_{0.0};
  double k_f_{0.0};
  double target_k_f_{0.0};
  double filter_gain_{0.001};
  Eigen::Matrix<double, 7, 1> tau_ext_initial_;

  // Dynamic reconfigure
  boost::scoped_ptr<
      dynamic_reconfigure::Server<franka_example_controllers::desired_mass_paramConfig>>
      dynamic_server_desired_mass_param_;
  ros::NodeHandle dynamic_reconfigure_desired_mass_param_node_;
  void desiredMassParamCallback(franka_example_controllers::desired_mass_paramConfig& config,
                                uint32_t level);
};

}  // namespace franka_example_controllers
