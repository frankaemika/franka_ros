// Copyright (c) 2019 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

#include <controller_manager/controller_manager.h>
#include <franka_hw/franka_combined_hw.h>
#include <ros/ros.h>

#include <sched.h>
#include <string>

int main(int argc, char** argv) {
  ros::init(argc, argv, "franka_combined_control_node");

  ros::AsyncSpinner spinner(4);
  spinner.start();

  ros::NodeHandle private_node_handle("~");
  franka_hw::FrankaCombinedHW franka_control;
  if (!franka_control.init(private_node_handle, private_node_handle)) {
    ROS_ERROR("franka_combined_control_node:: Initialization of FrankaCombinedHW failed!");
    return 1;
  }

  // set current control_loop thread to real-time
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

  controller_manager::ControllerManager cm(&franka_control, private_node_handle);
  ros::Duration period(0.001);
  ros::Rate rate(period);

  while (ros::ok()) {
    rate.sleep();
    franka_control.read(ros::Time::now(), period);
    cm.update(ros::Time::now(), period, franka_control.controllerNeedsReset());
    franka_control.write(ros::Time::now(), period);
  }

  return 0;
}
