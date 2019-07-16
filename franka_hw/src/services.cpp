// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_hw/services.h>

namespace franka_hw {

void setupServices(franka::Robot& robot, ros::NodeHandle& node_handle, ServiceContainer& services) {
  services
      .advertiseService<franka_msgs::SetJointImpedance>(node_handle, "set_joint_impedance",
                                                        [&robot](auto&& req, auto&& res) {
                                                          return franka_hw::setJointImpedance(
                                                              robot, req, res);
                                                        })
      .advertiseService<franka_msgs::SetCartesianImpedance>(
          node_handle, "set_cartesian_impedance",
          [&robot](auto&& req, auto&& res) {
            return franka_hw::setCartesianImpedance(robot, req, res);
          })
      .advertiseService<franka_msgs::SetEEFrame>(
          node_handle, "set_EE_frame",
          [&robot](auto&& req, auto&& res) { return franka_hw::setEEFrame(robot, req, res); })
      .advertiseService<franka_msgs::SetKFrame>(
          node_handle, "set_K_frame",
          [&robot](auto&& req, auto&& res) { return franka_hw::setKFrame(robot, req, res); })
      .advertiseService<franka_msgs::SetForceTorqueCollisionBehavior>(
          node_handle, "set_force_torque_collision_behavior",
          [&robot](auto&& req, auto&& res) {
            return franka_hw::setForceTorqueCollisionBehavior(robot, req, res);
          })
      .advertiseService<franka_msgs::SetFullCollisionBehavior>(
          node_handle, "set_full_collision_behavior",
          [&robot](auto&& req, auto&& res) {
            return franka_hw::setFullCollisionBehavior(robot, req, res);
          })
      .advertiseService<franka_msgs::SetLoad>(
          node_handle, "set_load",
          [&robot](auto&& req, auto&& res) { return franka_hw::setLoad(robot, req, res); });
}

void setCartesianImpedance(franka::Robot& robot,
                           const franka_msgs::SetCartesianImpedance::Request& req,
                           franka_msgs::SetCartesianImpedance::Response& /* res */) {
  std::array<double, 6> cartesian_stiffness;
  std::copy(req.cartesian_stiffness.cbegin(), req.cartesian_stiffness.cend(),
            cartesian_stiffness.begin());
  robot.setCartesianImpedance(cartesian_stiffness);
}

void setJointImpedance(franka::Robot& robot,
                       const franka_msgs::SetJointImpedance::Request& req,
                       franka_msgs::SetJointImpedance::Response& /* res */) {
  std::array<double, 7> joint_stiffness;
  std::copy(req.joint_stiffness.cbegin(), req.joint_stiffness.cend(), joint_stiffness.begin());
  robot.setJointImpedance(joint_stiffness);
}

void setEEFrame(franka::Robot& robot,
                const franka_msgs::SetEEFrame::Request& req,
                franka_msgs::SetEEFrame::Response& /* res */) {
  std::array<double, 16> F_T_EE;  // NOLINT [readability-identifier-naming]
  std::copy(req.F_T_EE.cbegin(), req.F_T_EE.cend(), F_T_EE.begin());
  robot.setEE(F_T_EE);
}

void setKFrame(franka::Robot& robot,
               const franka_msgs::SetKFrame::Request& req,
               franka_msgs::SetKFrame::Response& /* res */) {
  std::array<double, 16> EE_T_K;  // NOLINT [readability-identifier-naming]
  std::copy(req.EE_T_K.cbegin(), req.EE_T_K.cend(), EE_T_K.begin());
  robot.setK(EE_T_K);
}

void setForceTorqueCollisionBehavior(
    franka::Robot& robot,
    const franka_msgs::SetForceTorqueCollisionBehavior::Request& req,
    franka_msgs::SetForceTorqueCollisionBehavior::Response& /* res */
) {
  std::array<double, 7> lower_torque_thresholds_nominal;
  std::copy(req.lower_torque_thresholds_nominal.cbegin(),
            req.lower_torque_thresholds_nominal.cend(), lower_torque_thresholds_nominal.begin());
  std::array<double, 7> upper_torque_thresholds_nominal;
  std::copy(req.upper_torque_thresholds_nominal.cbegin(),
            req.upper_torque_thresholds_nominal.cend(), upper_torque_thresholds_nominal.begin());
  std::array<double, 6> lower_force_thresholds_nominal;
  std::copy(req.lower_force_thresholds_nominal.cbegin(), req.lower_force_thresholds_nominal.cend(),
            lower_force_thresholds_nominal.begin());
  std::array<double, 6> upper_force_thresholds_nominal;
  std::copy(req.upper_force_thresholds_nominal.cbegin(), req.upper_force_thresholds_nominal.cend(),
            upper_force_thresholds_nominal.begin());

  robot.setCollisionBehavior(lower_torque_thresholds_nominal, upper_torque_thresholds_nominal,
                             lower_force_thresholds_nominal, upper_force_thresholds_nominal);
}

void setFullCollisionBehavior(franka::Robot& robot,
                              const franka_msgs::SetFullCollisionBehavior::Request& req,
                              franka_msgs::SetFullCollisionBehavior::Response& /* res */) {
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
            req.lower_torque_thresholds_nominal.cend(), lower_torque_thresholds_nominal.begin());
  std::array<double, 7> upper_torque_thresholds_nominal;
  std::copy(req.upper_torque_thresholds_nominal.cbegin(),
            req.upper_torque_thresholds_nominal.cend(), upper_torque_thresholds_nominal.begin());
  std::array<double, 6> lower_force_thresholds_acceleration;
  std::copy(req.lower_force_thresholds_acceleration.cbegin(),
            req.lower_force_thresholds_acceleration.cend(),
            lower_force_thresholds_acceleration.begin());
  std::array<double, 6> upper_force_thresholds_acceleration;
  std::copy(req.upper_force_thresholds_acceleration.cbegin(),
            req.upper_force_thresholds_acceleration.cend(),
            upper_force_thresholds_acceleration.begin());
  std::array<double, 6> lower_force_thresholds_nominal;
  std::copy(req.lower_force_thresholds_nominal.cbegin(), req.lower_force_thresholds_nominal.cend(),
            lower_force_thresholds_nominal.begin());
  std::array<double, 6> upper_force_thresholds_nominal;
  std::copy(req.upper_force_thresholds_nominal.cbegin(), req.upper_force_thresholds_nominal.cend(),
            upper_force_thresholds_nominal.begin());
  robot.setCollisionBehavior(lower_torque_thresholds_acceleration,
                             upper_torque_thresholds_acceleration, lower_torque_thresholds_nominal,
                             upper_torque_thresholds_nominal, lower_force_thresholds_acceleration,
                             upper_force_thresholds_acceleration, lower_force_thresholds_nominal,
                             upper_force_thresholds_nominal);
}

void setLoad(franka::Robot& robot,
             const franka_msgs::SetLoad::Request& req,
             franka_msgs::SetLoad::Response& /* res */) {
  double mass(req.mass);
  std::array<double, 3> F_x_center_load;  // NOLINT [readability-identifier-naming]
  std::copy(req.F_x_center_load.cbegin(), req.F_x_center_load.cend(), F_x_center_load.begin());
  std::array<double, 9> load_inertia;
  std::copy(req.load_inertia.cbegin(), req.load_inertia.cend(), load_inertia.begin());

  robot.setLoad(mass, F_x_center_load, load_inertia);
}

}  // namespace franka_hw
