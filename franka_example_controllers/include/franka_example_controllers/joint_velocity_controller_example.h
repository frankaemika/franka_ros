#pragma once

#include <vector>

#include <ros/time.h>
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>

namespace franka_example_controllers {

class JointVelocityExampleController : public controller_interface::MultiInterfaceController<
    hardware_interface::VelocityJointInterface>
{
public:
  JointVelocityExampleController();
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle &node_handle);
  void update(const ros::Time& time, const ros::Duration& period);
  void stopping(const ros::Time& time);

private:
  std::vector<std::string> joint_names_;
  hardware_interface::VelocityJointInterface* velocity_joint_interface_;
  std::vector<hardware_interface::JointHandle> velocity_joint_handles_;
  ros::Time start_time_stamp_;
};

}  // namespace franka_example_controllers
