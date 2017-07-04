#pragma once

#include <ros/ros.h>

#include <franka_hw/ErrorRecovery.h>
#include <franka_hw/SetCartesianImpedance.h>
#include <franka_hw/SetEEFrame.h>
#include <franka_hw/SetForceTorqueCollisionBehavior.h>
#include <franka_hw/SetFullCollisionBehavior.h>
#include <franka_hw/SetJointImpedance.h>
#include <franka_hw/SetKFrame.h>
#include <franka_hw/SetLoad.h>
#include <franka_hw/SetTimeScalingFactor.h>

namespace franka {
class Robot;
}

namespace franka_hw {

class ServiceServer {
 public:
  ServiceServer() = delete;
  ServiceServer(franka::Robot& robot, ros::NodeHandle& node_handle);
  ~ServiceServer() {}

 private:
  bool setCartesianImpedance(const SetCartesianImpedance::Request& req);

  bool setJointImpedance(const SetJointImpedance::Request& req);

  bool setEEFrame(const SetEEFrame::Request& req);

  bool setKFrame(const SetKFrame::Request& req);

  bool setForceTorqueCollisionBehavior(
      const SetForceTorqueCollisionBehavior::Request& req);

  bool setFullCollisionBehavior(const SetFullCollisionBehavior::Request& req);

  bool setLoad(const SetLoad::Request& req);

  bool setTimeScalingFactor(const SetTimeScalingFactor::Request& req);

  bool errorRecovery();

  franka::Robot* robot_;
  ros::ServiceServer joint_impedance_server_;
  ros::ServiceServer cartesian_impedance_server_;
  ros::ServiceServer EE_frame_server_;
  ros::ServiceServer K_frame_server_;
  ros::ServiceServer force_torque_collision_server_;
  ros::ServiceServer full_collision_server_;
  ros::ServiceServer load_server_;
  ros::ServiceServer time_scaling_server_;
  ros::ServiceServer error_recovery_server_;
};

}  // namespace franka_hw
