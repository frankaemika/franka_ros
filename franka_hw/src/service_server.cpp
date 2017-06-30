#include <franka_hw/service_server.h>

#include <franka/robot.h>
#include <ros/ros.h>

#include <array>
#include <functional>

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
    std::function<void(T1&, T2&)> handler) {
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

ServiceServer::ServiceServer(franka::Robot* robot, ros::NodeHandle& node_handle)
    : robot_(robot),
      joint_impedance_server_(node_handle.advertiseService(
          "set_joint_impedance",
          createErrorFunction<SetJointImpedance::Request,
                              SetJointImpedance::Response>(
              std::bind(&ServiceServer::setJointImpedance,
                        this,
                        std::placeholders::_1,
                        std::placeholders::_2)))),

      cartesian_impedance_server_(node_handle.advertiseService(
          "set_cartesian_impedance",
          createErrorFunction<SetCartesianImpedance::Request,
                              SetCartesianImpedance::Response>(
              std::bind(&ServiceServer::setCartesianImpedance,
                        this,
                        std::placeholders::_1,
                        std::placeholders::_2)))),
      EE_frame_server_(node_handle.advertiseService(
          "set_EE_frame",
          createErrorFunction<SetEEFrame::Request, SetEEFrame::Response>(
              std::bind(&ServiceServer::setEEFrame,
                        this,
                        std::placeholders::_1,
                        std::placeholders::_2)))),
      K_frame_server_(node_handle.advertiseService(
          "set_K_frame",
          createErrorFunction<SetKFrame::Request, SetKFrame::Response>(
              std::bind(&ServiceServer::setKFrame,
                        this,
                        std::placeholders::_1,
                        std::placeholders::_2)))),
      force_torque_collision_server_(node_handle.advertiseService(
          "set_force_torque_collision_behavior",
          createErrorFunction<SetForceTorqueCollisionBehavior::Request,
                              SetForceTorqueCollisionBehavior::Response>(
              std::bind(&ServiceServer::setForceTorqueCollisionBehavior,
                        this,
                        std::placeholders::_1,
                        std::placeholders::_2)))),
      full_collision_server_(node_handle.advertiseService(
          "set_full_collision_behavior",
          createErrorFunction<SetFullCollisionBehavior::Request,
                              SetFullCollisionBehavior::Response>(
              std::bind(&ServiceServer::setFullCollisionBehavior,
                        this,
                        std::placeholders::_1,
                        std::placeholders::_2)))),
      load_server_(node_handle.advertiseService(
          "set_load",
          createErrorFunction<SetLoad::Request, SetLoad::Response>(
              std::bind(&ServiceServer::setLoad,
                        this,
                        std::placeholders::_1,
                        std::placeholders::_2)))),
      time_scaling_server_(node_handle.advertiseService(
          "set_time_scaling_factor",
          createErrorFunction<SetTimeScalingFactor::Request,
                              SetTimeScalingFactor::Response>(
              std::bind(&ServiceServer::setTimeScalingFactor,
                        this,
                        std::placeholders::_1,
                        std::placeholders::_2)))),
      error_recovery_server_(node_handle.advertiseService(
          "error_recovery",
          createErrorFunction<ErrorRecovery::Request, ErrorRecovery::Response>(
              std::bind(&ServiceServer::errorRecovery,
                        this,
                        std::placeholders::_1,
                        std::placeholders::_2)))) {}

bool ServiceServer::setCartesianImpedance(
    SetCartesianImpedance::Request& req,
    SetCartesianImpedance::Response& res) {
  std::array<double, 6> cartesian_stiffness;
  std::copy(req.cartesian_stiffness.cbegin(), req.cartesian_stiffness.cend(),
            cartesian_stiffness.begin());
  robot_->setCartesianImpedance(cartesian_stiffness);
  return true;
}

bool ServiceServer::setJointImpedance(SetJointImpedance::Request& req,
                                      SetJointImpedance::Response& res) {
  std::array<double, 7> joint_stiffness;
  std::copy(req.joint_stiffness.cbegin(), req.joint_stiffness.cend(),
            joint_stiffness.begin());
  robot_->setJointImpedance(joint_stiffness);
  return true;
}

bool ServiceServer::setEEFrame(SetEEFrame::Request& req,
                               SetEEFrame::Response& res) {
  std::array<double, 16> F_T_EE;
  std::copy(req.F_T_EE.cbegin(), req.F_T_EE.cend(), F_T_EE.begin());
  robot_->setEE(F_T_EE);
  return true;
}

bool ServiceServer::setKFrame(SetKFrame::Request& req,
                              SetKFrame::Response& res) {
  std::array<double, 16> EE_T_K;
  std::copy(req.EE_T_K.cbegin(), req.EE_T_K.cend(), EE_T_K.begin());
  robot_->setK(EE_T_K);
  return true;
}

bool ServiceServer::setForceTorqueCollisionBehavior(
    SetForceTorqueCollisionBehavior::Request& req,
    SetForceTorqueCollisionBehavior::Response& res) {
  std::array<double, 7> lower_torque_thresholds_nominal;
  std::copy(req.lower_torque_thresholds_nominal.cbegin(),
            req.lower_torque_thresholds_nominal.cend(),
            lower_torque_thresholds_nominal.begin());
  std::array<double, 7> upper_torque_thresholds_nominal;
  std::copy(req.upper_torque_thresholds_nominal.cbegin(),
            req.upper_torque_thresholds_nominal.cend(),
            upper_torque_thresholds_nominal.begin());
  std::array<double, 6> lower_force_thresholds_nominal;
  std::copy(req.lower_force_thresholds_nominal.cbegin(),
            req.lower_force_thresholds_nominal.cend(),
            lower_force_thresholds_nominal.begin());
  std::array<double, 6> upper_force_thresholds_nominal;
  std::copy(req.upper_force_thresholds_nominal.cbegin(),
            req.upper_force_thresholds_nominal.cend(),
            upper_force_thresholds_nominal.begin());

  robot_->setCollisionBehavior(
      lower_torque_thresholds_nominal, upper_torque_thresholds_nominal,
      lower_force_thresholds_nominal, upper_force_thresholds_nominal);
  return true;
}

bool ServiceServer::setFullCollisionBehavior(
    SetFullCollisionBehavior::Request& req,
    SetFullCollisionBehavior::Response& res) {
  std::array<double, 7> lower_torque_thresholds_acceleration;
  std::copy(req.lower_torque_thresholds_acceleration.cbegin(),
            req.lower_torque_thresholds_acceleration.cend(),
            lower_torque_thresholds_acceleration.begin());
  std::array<double, 7> upper_torque_thresholds_acceleration;
  std::copy(req.upper_torque_thresholds_acceleration.cbegin(),
            req.upper_torque_thresholds_acceleration.cend(),
            upper_torque_thresholds_acceleration.begin());
  std::array<double, 7> lower_torque_thresholds_nominal;
  std::copy(req.lower_torque_thresholds_nominal.cbegin(),
            req.lower_torque_thresholds_nominal.cend(),
            lower_torque_thresholds_nominal.begin());
  std::array<double, 7> upper_torque_thresholds_nominal;
  std::copy(req.upper_torque_thresholds_nominal.cbegin(),
            req.upper_torque_thresholds_nominal.cend(),
            upper_torque_thresholds_nominal.begin());
  std::array<double, 6> lower_force_thresholds_acceleration;
  std::copy(req.lower_force_thresholds_acceleration.cbegin(),
            req.lower_force_thresholds_acceleration.cend(),
            lower_force_thresholds_acceleration.begin());
  std::array<double, 6> upper_force_thresholds_acceleration;
  std::copy(req.upper_force_thresholds_acceleration.cbegin(),
            req.upper_force_thresholds_acceleration.cend(),
            upper_force_thresholds_acceleration.begin());
  std::array<double, 6> lower_force_thresholds_nominal;
  std::copy(req.lower_force_thresholds_nominal.cbegin(),
            req.lower_force_thresholds_nominal.cend(),
            lower_force_thresholds_nominal.begin());
  std::array<double, 6> upper_force_thresholds_nominal;
  std::copy(req.upper_force_thresholds_nominal.cbegin(),
            req.upper_force_thresholds_nominal.cend(),
            upper_force_thresholds_nominal.begin());
  robot_->setCollisionBehavior(
      lower_torque_thresholds_acceleration,
      upper_torque_thresholds_acceleration, lower_torque_thresholds_nominal,
      upper_torque_thresholds_nominal, lower_force_thresholds_acceleration,
      upper_force_thresholds_acceleration, lower_force_thresholds_nominal,
      upper_force_thresholds_nominal);
  return true;
}

bool ServiceServer::setLoad(SetLoad::Request& req, SetLoad::Response& res) {
  double mass(req.mass);
  std::array<double, 3> F_x_center_load;
  std::copy(req.F_x_center_load.cbegin(), req.F_x_center_load.cend(),
            F_x_center_load.begin());
  std::array<double, 9> load_inertia;
  std::copy(req.load_inertia.cbegin(), req.load_inertia.cend(),
            load_inertia.begin());

  robot_->setLoad(mass, F_x_center_load, load_inertia);
  return true;
}

bool ServiceServer::setTimeScalingFactor(SetTimeScalingFactor::Request& req,
                                         SetTimeScalingFactor::Response& res) {
  robot_->setTimeScalingFactor(req.time_scaling_factor);
  return true;
}

bool ServiceServer::errorRecovery(ErrorRecovery::Request& req,
                                  ErrorRecovery::Response& res) {
  robot_->automaticErrorRecovery();
  return true;
}

}  // namespace franka_hw
