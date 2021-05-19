#pragma once

#include <array>

#include <franka/model.h>
#include <franka_hw/model_base.h>

namespace franka_hw {

/**
 * An implementation of the abstract ModelBase specialized for obtaining the model from the real
 * robot.
 *
 * This class is a thin wrapper around a @ref franka::Model and delegates all calls to
 * that
 */
class Model : public ModelBase {
 public:
  /**
   * Create a new Model instance wrapped around a franka::Model
   */
  Model(franka::Model&& model) : model_(std::move(model)) {}

  std::array<double, 16> pose(
      franka::Frame frame,
      const std::array<double, 7>& q,
      const std::array<double, 16>& F_T_EE,  // NOLINT(readability-identifier-naming)
      const std::array<double, 16>& EE_T_K)  // NOLINT(readability-identifier-naming)
      const override {
    return model_.pose(frame, q, F_T_EE, EE_T_K);
  }

  std::array<double, 42> bodyJacobian(
      franka::Frame frame,
      const std::array<double, 7>& q,
      const std::array<double, 16>& F_T_EE,  // NOLINT(readability-identifier-naming)
      const std::array<double, 16>& EE_T_K)  // NOLINT(readability-identifier-naming)
      const override {
    return model_.bodyJacobian(frame, q, F_T_EE, EE_T_K);
  }

  std::array<double, 42> zeroJacobian(
      franka::Frame frame,
      const std::array<double, 7>& q,
      const std::array<double, 16>& F_T_EE,  // NOLINT(readability-identifier-naming)
      const std::array<double, 16>& EE_T_K)  // NOLINT(readability-identifier-naming)
      const override {
    return model_.zeroJacobian(frame, q, F_T_EE, EE_T_K);
  }

  std::array<double, 49> mass(
      const std::array<double, 7>& q,
      const std::array<double, 9>& I_total,  // NOLINT(readability-identifier-naming)
      double m_total,
      const std::array<double, 3>& F_x_Ctotal)  // NOLINT(readability-identifier-naming)
      const noexcept override {
    return model_.mass(q, I_total, m_total, F_x_Ctotal);
  }

  std::array<double, 7> coriolis(
      const std::array<double, 7>& q,
      const std::array<double, 7>& dq,
      const std::array<double, 9>& I_total,  // NOLINT(readability-identifier-naming)
      double m_total,
      const std::array<double, 3>& F_x_Ctotal)  // NOLINT(readability-identifier-naming)
      const noexcept override {
    return model_.coriolis(q, dq, I_total, m_total, F_x_Ctotal);
  }

  std::array<double, 7> gravity(
      const std::array<double, 7>& q,
      double m_total,
      const std::array<double, 3>& F_x_Ctotal,  // NOLINT(readability-identifier-naming)
      const std::array<double, 3>& gravity_earth) const noexcept override {
    return model_.gravity(q, m_total, F_x_Ctotal, gravity_earth);
  }

 private:
  franka::Model model_;
};

}  // namespace franka_hw
