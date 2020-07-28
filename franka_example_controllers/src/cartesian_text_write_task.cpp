// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
// Modified by (M.Khesbak july 2020)
#include <controller_interface/controller_base.h>
#include <franka/gripper.h>
#include <franka_example_controllers/cartesian_text_write_task.h>
#include <hardware_interface/hardware_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/node_handle.h>
#include <ros/ros.h>
#include <thread>

namespace franka_example_controllers {
double xxx, yyy, zzz;
double xx_home, yy_home, zz_home;

bool CartesianTextWritingTask::init(hardware_interface::RobotHW* robot_hardware,
                                    ros::NodeHandle& node_handle) {
  cartesian_pose_interface_ = robot_hardware->get<franka_hw::FrankaPoseCartesianInterface>();

  if (cartesian_pose_interface_ == nullptr) {
    ROS_ERROR(
        "CartesianTextWritingTask: Could not get Cartesian Pose "
        "interface from hardware");
    return false;
  }

  std::string arm_id;

  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("CartesianTextWritingTask: Could not get parameter arm_id");
    return false;
  }

  try {
    cartesian_pose_handle_ = std::make_unique<franka_hw::FrankaCartesianPoseHandle>(
        cartesian_pose_interface_->getHandle(arm_id + "_robot"));
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM("CartesianTextWritingTask: Exception getting Cartesian handle: " << e.what());
    return false;
  }

  auto state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
  // ************************************************************************************************
  if (state_interface == nullptr) {
    ROS_ERROR("CartesianTextWritingTask: Could not get state interface from hardware");
    return false;
  }

  try {
    auto state_handle = state_interface->getHandle(arm_id + "_robot");

    std::array<double, 7> q_start{{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    for (size_t i = 0; i < q_start.size(); i++) {
      if (std::abs(state_handle.getRobotState().q_d[i] - q_start[i]) > 0.1) {
        ROS_ERROR_STREAM(
            "CartesianTextWritingTask: Robot is not in the expected starting position for "
            "running this example. Run `roslaunch franka_example_controllers move_to_start.launch "
            "robot_ip:=<robot-ip> load_gripper:=<has-attached-gripper>` first.");
        return false;
      }
    }
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM("CartesianTextWritingTask: Exception getting state handle: " << e.what());
    return false;
  }

  return true;
}

void CartesianTextWritingTask::starting(const ros::Time& /* time */) {
  //   You may initiate any setting here.
}
double step = 1;
double check = 0;
double line = 0;

void CartesianTextWritingTask::update(const ros::Time& /* time */, const ros::Duration& period) {
  std::array<double, 16> new_pose;
  double x, y, z, x_d, y_d, z_d, angle, delta_x, delta_y, delta_z, ampl, f, x_mid, y_mid, z_mid;

  //   these steps are the conditions for writting text (IAI) line by line.

  if (line == 1) {  // Line 1 drawing  (the letter  I )
    x_d = -0.12, y_d = 0, z_d = 0;
  }
  if (line == 2) {  // lifting the pen from the paper
    x_d = 0, y_d = 0, z_d = 0.01;
  }
  if (line == 3) {  // Puting the pen to the paper to draw next letter
    x_d = 0, y_d = -0.03, z_d = -0.01;
  }
  if (line == 4) {  // Line 4 drawing  (the letter  A )
    x_d = 0.12, y_d = -0.05, z_d = 0;
  }
  if (line == 5) {  // Line 4 drawing  (the letter  A )
    x_d = -0.12, y_d = -0.05, z_d = 0;
  }
  if (line == 6) {  // lifting the pen from the paper
    x_d = 0, y_d = 0, z_d = 0.01;
  }
  if (line == 7) {  // Moving to draw next letter
    x_d = 0, y_d = -0.03, z_d = 0;
  }
  if (line == 8) {  // Puting the pen to the paper to draw next letter
    x_d = 0, y_d = 0, z_d = -0.012;
  }
  if (line == 9) {  // Line 9 drawing  (the letter  I again)
    x_d = 0.12, y_d = 0, z_d =0;
  }
  if (line == 10) {  // lifting the pen from the paper
    x_d = 0, y_d = 0, z_d = 0.01;
  }
  if (line == 11) {  // moving the pen again to complete the letter  A
    x_d = -0.06, y_d = 0.1053, z_d = 0;
  }
  if (line == 12) {  // moving the pen to start of the letter  A
    x_d = 0, y_d = 0, z_d = -0.01;
  }
  if (line == 13) {  // completing the letter  A
    x_d = 0, y_d = -0.049, z_d = 0;
  }

  if (line == 15) {  //   condition for terminating writing (Last step)
    step = 3;
    //exit(0);       
  }

  if (step == 1) {  // step 2 is the code for implementing line drawing
    if (check == 0) {
      initial_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;
      elapsed_time_ = ros::Duration(0.0);
      check = 1;
      new_pose = initial_pose_;
      xxx = new_pose[12];
      yyy = new_pose[13];
      zzz = new_pose[14];
      if (line == 0) {
        xx_home = xxx;  // xx_home, yy_home, zz_home are the home point of the gripper (Saved to
                        // return the Arm to Home position)
        yy_home = yyy;
        zz_home = zzz;
      }
    }

    x = xxx + x_d;  //  normal axis increament (line by line)
    y = yyy + y_d;
    z = zzz + z_d;

    if (line == 14) {  // setting hOME coordinates
      x = xx_home;
      y = yy_home;
      z = zz_home;
    }
    if (line == 0) {  // setting the coordinates to start writing 8first step ever)
      x = 0.6;
      y = -0.1;
      z = 0.14;
    }
    elapsed_time_ += period;
    f = 1;

    angle = M_PI / 4 * (1 - std::cos(M_PI / 5.0 * elapsed_time_.toSec() / f));
    delta_x = (x - xxx) * std::sin(angle);
    delta_y = (y - yyy) * std::sin(angle);
    delta_z = (z - zzz) * std::sin(angle);
    ampl = std::sin(angle);
    new_pose = initial_pose_;
    new_pose[12] += delta_x;
    new_pose[13] += delta_y;
    new_pose[14] += delta_z;
    cartesian_pose_handle_->setCommand(new_pose);

    // }
    if (ampl == 1) {
      line = line + 1;
      check = 0;
    }
  }
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::CartesianTextWritingTask,
                       controller_interface::ControllerBase)
