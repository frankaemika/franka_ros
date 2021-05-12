#pragma once

#include <franka/robot_state.h>
#include <franka_hw/model_base.h>
#include <urdf/model.h>
#include <array>
#include <kdl/chaindynparam.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <memory>
#include <string>

namespace franka_gazebo {

/**
 * Calculates poses of joints and dynamic properties of the robot.
 */
class ModelKDL : public franka_hw::ModelBase {
 public:
  ModelKDL(const urdf::Model& model, std::string root, std::string tip);

  std::array<double, 16> pose(
      franka::Frame frame,
      const std::array<double, 7>& q,
      const std::array<double, 16>& F_T_EE,  // NOLINT(readability-identifier-naming)
      const std::array<double, 16>& EE_T_K)  // NOLINT(readability-identifier-naming)
      const override;

  std::array<double, 42> bodyJacobian(
      franka::Frame frame,
      const std::array<double, 7>& q,
      const std::array<double, 16>& F_T_EE,  // NOLINT(readability-identifier-naming)
      const std::array<double, 16>& EE_T_K)  // NOLINT(readability-identifier-naming)
      const override;

  std::array<double, 42> zeroJacobian(
      franka::Frame frame,
      const std::array<double, 7>& q,
      const std::array<double, 16>& F_T_EE,  // NOLINT(readability-identifier-naming)
      const std::array<double, 16>& EE_T_K)  // NOLINT(readability-identifier-naming)
      const override;

  std::array<double, 49> mass(
      const std::array<double, 7>& q,
      const std::array<double, 9>& I_total,  // NOLINT(readability-identifier-naming)
      double m_total,
      const std::array<double, 3>& F_x_Ctotal)  // NOLINT(readability-identifier-naming)
      const noexcept override;

  std::array<double, 7> coriolis(
      const std::array<double, 7>& q,
      const std::array<double, 7>& dq,
      const std::array<double, 9>& I_total,  // NOLINT(readability-identifier-naming)
      double m_total,
      const std::array<double, 3>& F_x_Ctotal)  // NOLINT(readability-identifier-naming)
      const noexcept override;

  std::array<double, 7> gravity(
      const std::array<double, 7>& q,
      double m_total,
      const std::array<double, 3>& F_x_Ctotal,  // NOLINT(readability-identifier-naming)
      const std::array<double, 3>& gravity_earth = {{0., 0., -9.81}}) const noexcept override;

 private:
  std::unique_ptr<KDL::ChainDynParam> dynamicsSolver_;
  std::unique_ptr<KDL::ChainJntToJacSolver> jacobianSolver_;
  std::unique_ptr<KDL::ChainFkSolverPos> kinematicsSolver_;
  KDL::Chain chain;
};

}  // namespace franka_gazebo
