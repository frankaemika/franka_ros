#include <franka_hw/service_server.h>

#include <franka/robot.h>
#include <ros/ros.h>

#include <array>

namespace {
template <typename T>
ros::ServiceServer advertiseService(
    ros::NodeHandle& node_handle,
    const std::string& name,
    std::function<void(typename T::Request&, typename T::Response&)> handler,
    const std::unordered_map<std::string, std::function<void()>>&
        success_callbacks) {
  std::function<void()> success_callback;
  auto it = success_callbacks.find(name);
  if (it != success_callbacks.cend()) {
    success_callback = it->second;
  }

  return node_handle.advertiseService<typename T::Request,
                                      typename T::Response>(
      name, [name, handler, success_callback](typename T::Request& request,
                                              typename T::Response& response) {
        try {
          handler(request, response);
          response.success = true;
          if (success_callback) {
            success_callback();
          }
          ROS_DEBUG_STREAM(name << " succeeded.");
        } catch (const franka::Exception& ex) {
          ROS_ERROR_STREAM(name << " failed: " << ex.what());
          response.success = false;
          response.error = ex.what();
        }
        return true;
      });
}

}  // anonymous namespace

namespace franka_hw {

using std::placeholders::_1;

ServiceServer::ServiceServer(
    franka::Robot& robot,
    ros::NodeHandle& node_handle,
    const std::unordered_map<std::string, std::function<void()>>&
        success_callbacks)
    : robot_(&robot),
      joint_impedance_server_(advertiseService<SetJointImpedance>(
          node_handle,
          "set_joint_impedance",
          std::bind(&ServiceServer::setJointImpedance, this, _1),
          success_callbacks)),
      cartesian_impedance_server_(advertiseService<SetCartesianImpedance>(
          node_handle,
          "set_cartesian_impedance",
          std::bind(&ServiceServer::setCartesianImpedance, this, _1),
          success_callbacks)),
      EE_frame_server_(advertiseService<SetEEFrame>(
          node_handle,
          "set_EE_frame",
          std::bind(&ServiceServer::setEEFrame, this, std::placeholders::_1),
          success_callbacks)),
      K_frame_server_(advertiseService<SetKFrame>(
          node_handle,
          "set_K_frame",
          std::bind(&ServiceServer::setKFrame, this, _1),
          success_callbacks)),
      force_torque_collision_server_(
          advertiseService<SetForceTorqueCollisionBehavior>(
              node_handle,
              "set_force_torque_collision_behavior",
              std::bind(&ServiceServer::setForceTorqueCollisionBehavior,
                        this,
                        _1),
              success_callbacks)),
      full_collision_server_(advertiseService<SetFullCollisionBehavior>(
          node_handle,
          "set_full_collision_behavior",
          std::bind(&ServiceServer::setFullCollisionBehavior, this, _1),
          success_callbacks)),
      load_server_(advertiseService<SetLoad>(
          node_handle,
          "set_load",
          std::bind(&ServiceServer::setLoad, this, _1),
          success_callbacks)),
      time_scaling_server_(advertiseService<SetTimeScalingFactor>(
          node_handle,
          "set_time_scaling_factor",
          std::bind(&ServiceServer::setTimeScalingFactor, this, _1),
          success_callbacks)),
      error_recovery_server_(advertiseService<ErrorRecovery>(
          node_handle,
          "error_recovery",
          std::bind(&ServiceServer::errorRecovery, this),
          success_callbacks)) {}

void ServiceServer::setCartesianImpedance(
    const SetCartesianImpedance::Request& req) {
  std::array<double, 6> cartesian_stiffness;
  std::copy(req.cartesian_stiffness.cbegin(), req.cartesian_stiffness.cend(),
            cartesian_stiffness.begin());
  robot_->setCartesianImpedance(cartesian_stiffness);
}

void ServiceServer::setJointImpedance(const SetJointImpedance::Request& req) {
  std::array<double, 7> joint_stiffness;
  std::copy(req.joint_stiffness.cbegin(), req.joint_stiffness.cend(),
            joint_stiffness.begin());
  robot_->setJointImpedance(joint_stiffness);
}

void ServiceServer::setEEFrame(const SetEEFrame::Request& req) {
  std::array<double, 16> F_T_EE;  // NOLINT [readability-identifier-naming]
  std::copy(req.F_T_EE.cbegin(), req.F_T_EE.cend(), F_T_EE.begin());
  robot_->setEE(F_T_EE);
}

void ServiceServer::setKFrame(const SetKFrame::Request& req) {
  std::array<double, 16> EE_T_K;  // NOLINT [readability-identifier-naming]
  std::copy(req.EE_T_K.cbegin(), req.EE_T_K.cend(), EE_T_K.begin());
  robot_->setK(EE_T_K);
}

void ServiceServer::setForceTorqueCollisionBehavior(
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
}

void ServiceServer::setFullCollisionBehavior(
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
}

void ServiceServer::setLoad(const SetLoad::Request& req) {
  double mass(req.mass);
  std::array<double, 3>
      F_x_center_load;  // NOLINT [readability-identifier-naming]
  std::copy(req.F_x_center_load.cbegin(), req.F_x_center_load.cend(),
            F_x_center_load.begin());
  std::array<double, 9> load_inertia;
  std::copy(req.load_inertia.cbegin(), req.load_inertia.cend(),
            load_inertia.begin());

  robot_->setLoad(mass, F_x_center_load, load_inertia);
}

void ServiceServer::setTimeScalingFactor(
    const SetTimeScalingFactor::Request& req) {
  robot_->setTimeScalingFactor(req.time_scaling_factor);
}

void ServiceServer::errorRecovery() {
  robot_->automaticErrorRecovery();
}

}  // namespace franka_hw
