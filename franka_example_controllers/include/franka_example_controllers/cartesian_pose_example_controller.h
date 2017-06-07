#pragma once

#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/robot_hw.h>
#include <ros/time.h>

#include <franka_hw/franka_cartesian_command_interface.h>

namespace franka_example_controllers {

class CartesianPoseExampleController
    : public controller_interface::MultiInterfaceController<
          franka_hw::FrankaPoseCartesianInterface> {
 public:
  CartesianPoseExampleController();
  bool init(hardware_interface::RobotHW* robot_hw,
            ros::NodeHandle& node_handle);
  void update(const ros::Time& time, const ros::Duration& period);

 private:
  std::string arm_id_;
  franka_hw::FrankaPoseCartesianInterface* cartesian_pose_interface_;
  std::array<double, 16> initial_pose_ = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                           0.0, 0.0}};
  ros::Duration elapsed_time_;
};

}  // namespace franka_example_controllers
