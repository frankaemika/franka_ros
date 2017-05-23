#pragma once

#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/time.h>

namespace franka_example_controllers {

class JointPositionExampleController
    : public controller_interface::MultiInterfaceController<
          hardware_interface::PositionJointInterface> {
 public:
  JointPositionExampleController();
  bool init(hardware_interface::RobotHW* robot_hw,
            ros::NodeHandle& node_handle);
  void update(const ros::Time& time, const ros::Duration& period);

 private:
  std::vector<std::string> joint_names_;
  hardware_interface::PositionJointInterface* position_joint_interface_;
  std::vector<hardware_interface::JointHandle> position_joint_handles_;
  std::array<double, 7> initial_pose_ = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  ros::Time start_time_stamp_;
};

}  // namespace franka_example_controllers
