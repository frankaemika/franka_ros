// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
// Modified by (M.Khesbak july 2020)
#include <actionlib/client/simple_action_client.h>
#include <controller_interface/controller_base.h>
#include <franka/gripper.h>
#include <franka_example_controllers/cartesian_pickup_task.h>
#include <franka_gripper/GraspAction.h>
#include <franka_gripper/MoveAction.h>
#include <hardware_interface/hardware_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/node_handle.h>
#include <ros/ros.h>
#include <thread>

namespace franka_example_controllers {
double xx, yy, zz;
double x_home, y_home, z_home, cnt1 = 0, cnt2 = 0;

bool CartesianPickupTask::init(hardware_interface::RobotHW* robot_hardware,
                               ros::NodeHandle& node_handle) {
  cartesian_pose_interface_ = robot_hardware->get<franka_hw::FrankaPoseCartesianInterface>();

  if (cartesian_pose_interface_ == nullptr) {
    ROS_ERROR(
        "CartesianPickupTask: Could not get Cartesian Pose "
        "interface from hardware");
    return false;
  }

  std::string arm_id;

  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("CartesianPickupTask: Could not get parameter arm_id");
    return false;
  }

  try {
    cartesian_pose_handle_ = std::make_unique<franka_hw::FrankaCartesianPoseHandle>(
        cartesian_pose_interface_->getHandle(arm_id + "_robot"));
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM("CartesianPickupTask: Exception getting Cartesian handle: " << e.what());
    return false;
  }

  auto state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
  // ************************************************************************************************
  if (state_interface == nullptr) {
    ROS_ERROR("CartesianPickupTask: Could not get state interface from hardware");
    return false;
  }

  try {
    auto state_handle = state_interface->getHandle(arm_id + "_robot");

    std::array<double, 7> q_start{{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    for (size_t i = 0; i < q_start.size(); i++) {
      if (std::abs(state_handle.getRobotState().q_d[i] - q_start[i]) > 0.1) {
        ROS_ERROR_STREAM(
            "CartesianPickupTask: Robot is not in the expected starting position for "
            "running this example. Run `roslaunch franka_example_controllers move_to_start.launch "
            "robot_ip:=<robot-ip> load_gripper:=<has-attached-gripper>` first.");
        return false;
      }
    }
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM("CartesianPickupTask: Exception getting state handle: " << e.what());
    return false;
  }

  return true;
}

void CartesianPickupTask::starting(const ros::Time& /* time */) {
  //   You can initialize any thing here.
}
double schritt = 1, F_x = 1, F_y = 1, F_z = 1;
double Pcheck = 0;
double move = 1;
// Defining the Grasp action variable (ac).
actionlib::SimpleActionClient<franka_gripper::GraspAction> ac("franka_gripper/grasp", true);
// Defining the gripper Move action variable (ac1).
actionlib::SimpleActionClient<franka_gripper::MoveAction> ac1("franka_gripper/move", true);

void CartesianPickupTask::update(const ros::Time& /* time */, const ros::Duration& period) {
  std::array<double, 16> new_pose;
  double x, y, z, angle, delta_x, delta_y, delta_z, ampl, f, Grasp_Width, Move_Width;

  //   These steps (moves) are the conditions for Arm movement steps and object Grasp and release.
  //   You may add any required steps to do more actions.
  if (move == 1) {
    x = 0.5, y = -0.45, z = 0.13;  // Go to the Object coordinations
    F_x = 1, F_y = 1, F_z = 1;
    Move_Width = 0.08;
    franka_gripper::MoveGoal goal1;
    goal1.width = Move_Width;
    goal1.speed = 0.1;  //  Closing speed. [m/s]
    ac1.sendGoal(goal1);
  }
  if (move == 2) {  // go down to pick up the object
    z = 0.09;
    F_x = 0, F_y = 0, F_z = 1;
  }

  if (move == 3 & schritt == 1) {  // grasp the object
    schritt = 2;
    Grasp_Width = 0.038;

    schritt = 2;
  }

  if (move == 4) {  // go up after grasping the object
    z = 0.35;
    F_x = 0, F_y = 0, F_z = 1;
  }
  if (move == 5) {  // Move the object to the new coordinations
    x = 0.6, y = 0, z = 0.09;
    F_x = 1, F_y = -1, F_z = 1;
  }
  if (move == 6 & schritt == 1) {  // release the object
    Grasp_Width = 0.08;
    schritt = 2;
  }
  if (move == 7) {  // go to HOME position after the object had been released
    x = x_home, y = y_home, z = z_home;
    F_x = 1, F_y = -1, F_z = 1;
  }

  if (move == 8) {  // Programm termination
    schritt = 9;    //  For repeatitive task please set schritt to (1).
    Pcheck = 0;
    move = 1;
    // exit(0);      // uncomment this for program termination
  }
  // ********************************************************************************************

  //  Schritt 1 = condition for implementing a certain Arm movement to required new (x,y,z)
  //  Schritt 2 = condition for Grasping an action or release it with certain width, speed and
  //  force.

  if (schritt == 1) {
    if (Pcheck == 0) {
      initial_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;
      elapsed_time_ = ros::Duration(0.0);
      Pcheck = 1;
      new_pose = initial_pose_;
      xx = new_pose[12];  // xx, yy, zz are the current point of the gripper
      yy = new_pose[13];
      zz = new_pose[14];
      if (move == 1) {
        x_home = xx;  // x_home, y_home, z_home are the home point of the gripper (Saved to return
                      // the Arm to Home position)
        y_home = yy;
        z_home = zz;
      }
    }
    elapsed_time_ += period;
    f = 1;  // Increasing (f) more than 1 will decrease the speed of the Arm
    // Calculating the angular angle for gradual motion
    angle = M_PI / 4 * (1 - std::cos(M_PI / 5.0 * elapsed_time_.toSec() / f));
    // Calculating the gradual motion increament for each axis
    delta_x = F_x * (x - xx) * std::sin(angle);
    delta_y = F_y * (y + yy) * std::sin(angle);
    delta_z = F_z * (z - zz) * std::sin(angle);
    ampl = std::sin(angle);
    new_pose = initial_pose_;
    new_pose[12] += delta_x;  // Updateing x-axis
    new_pose[13] += delta_y;  // Updateing y-axis
    new_pose[14] += delta_z;  // Updateing z-axis
    // Sending the new position to robot Arm
    cartesian_pose_handle_->setCommand(new_pose);
    if (ampl == 1) {  // Checking for velocity zero of the Arm (to stop movement)
      move = move + 1;
      Pcheck = 0;
    }
  }
  if (schritt == 2) {  //   grasp implementation condition
    franka_gripper::GraspGoal goal;
    goal.width = Grasp_Width;
    goal.speed = 0.1;           //  Closing speed. [m/s]
    goal.force = 60;            //   Grasping (continuous) force [N]
    goal.epsilon.inner = 0.05;  // Maximum tolerated deviation when the actual grasped width is
                                // smaller than the commanded grasp width.
    goal.epsilon.outer = 0.05;  // Maximum tolerated deviation when the actual grasped width is
                                // larger than the commanded grasp width.
    ac.sendGoal(goal);          // Sending the Grasp command to gripper
    schritt = 2.5;
  }
  if (schritt == 2.5) {  //   grasp delay and parameter update
    cnt1 = cnt1 + 1;

    if (cnt1 == 900) {
      move = move + 1;
      schritt = 1;
      Pcheck = 0;
      cnt1 = 0;
    }
  }
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::CartesianPickupTask,
                       controller_interface::ControllerBase)
