// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <array>
#include <memory>
#include <string>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>

#include <franka_hw/franka_cartesian_command_interface.h>

namespace franka_example_controllers {

class CartesianPoseExampleController : public controller_interface::MultiInterfaceController<
                                           franka_hw::FrankaPoseCartesianInterface> {
 public:
  CartesianPoseExampleController();
  bool init(hardware_interface::RobotHW* robot_hardware,
            ros::NodeHandle& root_node_handle,
            ros::NodeHandle&);
  void starting(const ros::Time&);

  void update(const ros::Time&, const ros::Duration& period);

 private:
  std::string arm_id_;
  franka_hw::FrankaPoseCartesianInterface* cartesian_pose_interface_;
  std::array<double, 16> initial_pose_ = {
      {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  ros::Duration elapsed_time_;
  std::unique_ptr<franka_hw::FrankaCartesianPoseHandle> cartesian_pose_handle_;
};

}  // namespace franka_example_controllers
