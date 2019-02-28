// Copyright (c) 2019 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

#include <controller_manager/controller_manager.h>
#include <franka_combinable_hw/franka_combined_hw.h>
#include <ros/ros.h>

#include <sched.h>
#include <stdexcept>

int main(int argc, char** argv) {
  ros::init(argc, argv, "franka_combined_control_node");

  // set current control_loop thread to read-time
  const int kThreadPriority = sched_get_priority_max(SCHED_FIFO);
  if (kThreadPriority == -1) {
    ROS_ERROR("franka_combined_control_node: unable to get maximum possible thread priority: %s",
              std::strerror(errno));
    return 1;
  }
  sched_param thread_param{};
  thread_param.sched_priority = kThreadPriority;
  if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &thread_param) != 0) {
    ROS_ERROR("franka_combined_control_node: unable to set realtime scheduling: %s",
              std::strerror(errno));
    return 1;
  }

  ros::AsyncSpinner spinner(4);
  spinner.start();

  ros::NodeHandle nh("~");
  franka_combinable_hw::FrankaCombinedHW hw;
  bool init_success = hw.init(nh, nh);

  if (!init_success) {
    throw std::runtime_error(
        "franka_combined_control_node:: Initialization of FrankaCombinedHW failed!");
    return 1;
  }

  controller_manager::ControllerManager cm(&hw, nh);

  ros::Duration period(0.001);
  while (ros::ok()) {
    hw.read(ros::Time::now(), period);
    cm.update(ros::Time::now(), period, hw.controllerNeedsReset());
    hw.write(ros::Time::now(), period);
    period.sleep();
  }

  return 0;
}
