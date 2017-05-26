#pragma once

#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/time.h>

namespace franka_example_controllers {

class CartesianVelocityExampleController
    : public controller_interface::MultiInterfaceController<
          franka_hw::FrankaVelocityCartesianInterface> {
 public:
  CartesianVelocityExampleController();
  bool init(hardware_interface::RobotHW* robot_hw,
            ros::NodeHandle& node_handle);
  void update(const ros::Time& time, const ros::Duration& period);
  void stopping(const ros::Time& time);

 private:
  std::string arm_id_;
  franka_hw::FrankaVelocityCartesianInterface* velocity_cartesian_interface_;
  ros::Duration elapsed_time_;
};

}  // namespace franka_example_controllers
