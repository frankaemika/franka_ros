#pragma once

#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <pluginlib/class_list_macros.h>
#include <urdf/model.h>

namespace test_franka_hw {

class TestJointTorqueLimitsController
    : public controller_interface::MultiInterfaceController<
          hardware_interface::EffortJointInterface> {
 public:
  TestJointTorqueLimitsController();
  bool init(hardware_interface::RobotHW* robot_hw,
            ros::NodeHandle& node_handle);
  void update(const ros::Time& time, const ros::Duration& period);

 private:
  std::vector<joint_limits_interface::JointLimits> joint_limits_;
  std::vector<std::string> joint_names_;
  hardware_interface::EffortJointInterface* effort_interface_;
  std::uniform_real_distribution<double> uniform_distribution_;
  std::default_random_engine random_engine_;
  bool phase_ = false;
  std::vector<hardware_interface::JointHandle> effort_joint_handles_;
};

}  // namespace test_franka_hw
