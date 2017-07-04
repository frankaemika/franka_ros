#include <franka_hw/service_server.h>

#include <franka/robot.h>
#include <ros/ros.h>

#include <array>
#include <boost/function.hpp>
#include <functional>

namespace {
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

}  // anonymous namespace

namespace franka_hw {

using std::placeholders::_1;

ServiceServer::ServiceServer(franka::Robot& robot, ros::NodeHandle& node_handle)
    : robot_(&robot),
      joint_impedance_server_(node_handle.advertiseService(
          "set_joint_impedance",
          createErrorFunction<SetJointImpedance::Request,
                              SetJointImpedance::Response>(
              std::bind(&ServiceServer::setJointImpedance, this, _1)))),

      cartesian_impedance_server_(node_handle.advertiseService(
          "set_cartesian_impedance",
          createErrorFunction<SetCartesianImpedance::Request,
                              SetCartesianImpedance::Response>(
              std::bind(&ServiceServer::setCartesianImpedance, this, _1)))),
      EE_frame_server_(node_handle.advertiseService(
          "set_EE_frame",
          createErrorFunction<SetEEFrame::Request, SetEEFrame::Response>(
              std::bind(&ServiceServer::setEEFrame,
                        this,
                        std::placeholders::_1)))),
      K_frame_server_(node_handle.advertiseService(
          "set_K_frame",
          createErrorFunction<SetKFrame::Request, SetKFrame::Response>(
              std::bind(&ServiceServer::setKFrame, this, _1)))),
      force_torque_collision_server_(node_handle.advertiseService(
          "set_force_torque_collision_behavior",
          createErrorFunction<SetForceTorqueCollisionBehavior::Request,
                              SetForceTorqueCollisionBehavior::Response>(
              std::bind(&ServiceServer::setForceTorqueCollisionBehavior,
                        this,
                        _1)))),
      full_collision_server_(node_handle.advertiseService(
          "set_full_collision_behavior",
          createErrorFunction<SetFullCollisionBehavior::Request,
                              SetFullCollisionBehavior::Response>(
              std::bind(&ServiceServer::setFullCollisionBehavior, this, _1)))),
      load_server_(node_handle.advertiseService(
          "set_load",
          createErrorFunction<SetLoad::Request, SetLoad::Response>(
              std::bind(&ServiceServer::setLoad, this, _1)))),
      time_scaling_server_(node_handle.advertiseService(
          "set_time_scaling_factor",
          createErrorFunction<SetTimeScalingFactor::Request,
                              SetTimeScalingFactor::Response>(
              std::bind(&ServiceServer::setTimeScalingFactor, this, _1)))),
      error_recovery_server_(node_handle.advertiseService(
          "error_recovery",
          createErrorFunction<ErrorRecovery::Request, ErrorRecovery::Response>(
              std::bind(&ServiceServer::errorRecovery, this)))) {}

bool ServiceServer::setCartesianImpedance(
    const SetCartesianImpedance::Request& req) {
  std::array<double, 6> cartesian_stiffness;
  std::copy(req.cartesian_stiffness.cbegin(), req.cartesian_stiffness.cend(),
            cartesian_stiffness.begin());
  robot_->setCartesianImpedance(cartesian_stiffness);
  return true;
}

bool ServiceServer::setJointImpedance(const SetJointImpedance::Request& req) {
  std::array<double, 7> joint_stiffness;
  std::copy(req.joint_stiffness.cbegin(), req.joint_stiffness.cend(),
            joint_stiffness.begin());
  robot_->setJointImpedance(joint_stiffness);
  return true;
}

bool ServiceServer::setEEFrame(const SetEEFrame::Request& req) {
  std::array<double, 16> F_T_EE;  // NOLINT [readability-identifier-naming]
  std::copy(req.F_T_EE.cbegin(), req.F_T_EE.cend(), F_T_EE.begin());
  robot_->setEE(F_T_EE);
  return true;
}

bool ServiceServer::setKFrame(const SetKFrame::Request& req) {
  std::array<double, 16> EE_T_K;  // NOLINT [readability-identifier-naming]
  std::copy(req.EE_T_K.cbegin(), req.EE_T_K.cend(), EE_T_K.begin());
  robot_->setK(EE_T_K);
  return true;
}

bool ServiceServer::setForceTorqueCollisionBehavior(
    const SetForceTorqueCollisionBehavior::Request& req) {
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
    const SetFullCollisionBehavior::Request& req) {
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

bool ServiceServer::setLoad(const SetLoad::Request& req) {
  double mass(req.mass);
  std::array<double, 3>
      F_x_center_load;  // NOLINT [readability-identifier-naming]
  std::copy(req.F_x_center_load.cbegin(), req.F_x_center_load.cend(),
            F_x_center_load.begin());
  std::array<double, 9> load_inertia;
  std::copy(req.load_inertia.cbegin(), req.load_inertia.cend(),
            load_inertia.begin());

  robot_->setLoad(mass, F_x_center_load, load_inertia);
  return true;
}

bool ServiceServer::setTimeScalingFactor(
    const SetTimeScalingFactor::Request& req) {
  robot_->setTimeScalingFactor(req.time_scaling_factor);
  return true;
}

bool ServiceServer::errorRecovery() {
  robot_->automaticErrorRecovery();
  return true;
}

}  // namespace franka_hw
