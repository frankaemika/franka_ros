// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_control/services.h>

namespace franka_control {

void setCartesianImpedance(franka::Robot& robot,
                           const SetCartesianImpedance::Request& req,
                           SetCartesianImpedance::Response& /* res */) {
  std::array<double, 6> cartesian_stiffness;
  std::copy(req.cartesian_stiffness.cbegin(), req.cartesian_stiffness.cend(),
            cartesian_stiffness.begin());
  robot.setCartesianImpedance(cartesian_stiffness);
}

void setJointImpedance(franka::Robot& robot,
                       const SetJointImpedance::Request& req,
                       SetJointImpedance::Response& /* res */) {
  std::array<double, 7> joint_stiffness;
  std::copy(req.joint_stiffness.cbegin(), req.joint_stiffness.cend(), joint_stiffness.begin());
  robot.setJointImpedance(joint_stiffness);
}

void setEEFrame(franka::Robot& robot,
                const SetEEFrame::Request& req,
                SetEEFrame::Response& /* res */) {
  std::array<double, 16> F_T_EE;  // NOLINT [readability-identifier-naming]
  std::copy(req.F_T_EE.cbegin(), req.F_T_EE.cend(), F_T_EE.begin());
  robot.setEE(F_T_EE);
}

void setKFrame(franka::Robot& robot,
               const SetKFrame::Request& req,
               SetKFrame::Response& /* res */) {
  std::array<double, 16> EE_T_K;  // NOLINT [readability-identifier-naming]
  std::copy(req.EE_T_K.cbegin(), req.EE_T_K.cend(), EE_T_K.begin());
  robot.setK(EE_T_K);
}

void setForceTorqueCollisionBehavior(franka::Robot& robot,
                                     const SetForceTorqueCollisionBehavior::Request& req,
                                     SetForceTorqueCollisionBehavior::Response& /* res */
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
                              const SetFullCollisionBehavior::Request& req,
                              SetFullCollisionBehavior::Response& /* res */) {
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

void setLoad(franka::Robot& robot, const SetLoad::Request& req, SetLoad::Response& /* res */) {
  double mass(req.mass);
  std::array<double, 3> F_x_center_load;  // NOLINT [readability-identifier-naming]
  std::copy(req.F_x_center_load.cbegin(), req.F_x_center_load.cend(), F_x_center_load.begin());
  std::array<double, 9> load_inertia;
  std::copy(req.load_inertia.cbegin(), req.load_inertia.cend(), load_inertia.begin());

  robot.setLoad(mass, F_x_center_load, load_inertia);
}

}  // namespace franka_control
