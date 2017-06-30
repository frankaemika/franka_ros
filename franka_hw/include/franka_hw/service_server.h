#pragma once

#include <functional>

#include <franka/exception.h>
#include <franka/robot.h>
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

namespace franka_hw {

template <typename T1, typename T2>
const boost::function<bool(T1&, T2&)> createErrorFunction(
    std::function<bool(T1&, T2&)> handler) {
  return [handler](T1& request, T2& response) -> bool {
    try {
      handler(request, response);
      response.success = true;
    } catch (const franka::Exception& ex) {
      ROS_ERROR_STREAM("" << ex.what());
      response.success = false;
      response.error = ex.what();
    }
    return true;
  };
}

class ServiceServer {
 public:
  ServiceServer() = delete;
  ServiceServer(franka::Robot* robot, ros::NodeHandle& node_handle);
  ~ServiceServer() {}

 private:
  bool setCartesianImpedance(SetCartesianImpedance::Request& req,
                             SetCartesianImpedance::Response& res);

  bool setJointImpedance(SetJointImpedance::Request& req,
                         SetJointImpedance::Response& res);

  bool setEEFrame(SetEEFrame::Request& req, SetEEFrame::Response& res);

  bool setKFrame(SetKFrame::Request& req, SetKFrame::Response& res);

  bool setForceTorqueCollisionBehavior(
      SetForceTorqueCollisionBehavior::Request& req,
      SetForceTorqueCollisionBehavior::Response& res);

  bool setFullCollisionBehavior(SetFullCollisionBehavior::Request& req,
                                SetFullCollisionBehavior::Response& res);

  bool setLoad(SetLoad::Request& req, SetLoad::Response& res);

  bool setTimeScalingFactor(SetTimeScalingFactor::Request& req,
                            SetTimeScalingFactor::Response& res);

  bool errorRecovery(ErrorRecovery::Request& req, ErrorRecovery::Response& res);

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
