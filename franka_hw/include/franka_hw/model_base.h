#pragma once

#include <franka/model.h>
#include <franka/robot_state.h>

namespace franka_hw {

/**
 * Calculates poses of joints and dynamic properties of the robot.
 *
 * This abstract class serves as an extendable interface for different
 * implementations. To get the dynamic model from a real robot refer to
 * @see franka_hw::Model.
 *
 * @note
 * Although this class is abstract, it offers default implementations
 * for all methods which take a @ref RobotState. The default implementation
 * simply delegates the class to their pure virtual counterparts.
 */
class ModelBase {
 public:
  virtual ~ModelBase() noexcept = default;

  /**
   * Gets the 4x4 pose matrix for the given frame in base frame.
   *
   * The pose is represented as a 4x4 matrix in column-major format.
   *
   * @param[in] frame The desired frame.
   * @param[in] robot_state State from which the pose should be calculated.
   *
   * @return Vectorized 4x4 pose matrix, column-major.
   */
  std::array<double, 16> pose(franka::Frame frame, const franka::RobotState& robot_state) const {
    return pose(frame, robot_state.q, robot_state.F_T_EE, robot_state.EE_T_K);
  }

  /**
   * Gets the 4x4 pose matrix for the given frame in base frame.
   *
   * The pose is represented as a 4x4 matrix in column-major format.
   *
   * @param[in] frame The desired frame.
   * @param[in] q Joint position.
   * @param[in] F_T_EE End effector in flange frame.
   * @param[in] EE_T_K Stiffness frame K in the end effector frame.
   *
   * @return Vectorized 4x4 pose matrix, column-major.
   */
  virtual std::array<double, 16> pose(
      franka::Frame frame,
      const std::array<double, 7>& q,
      const std::array<double, 16>& F_T_EE,  // NOLINT(readability-identifier-naming)
      const std::array<double, 16>& EE_T_K)  // NOLINT(readability-identifier-naming)
      const = 0;

  /**
   * Gets the 6x7 Jacobian for the given frame, relative to that frame.
   *
   * The Jacobian is represented as a 6x7 matrix in column-major format.
   *
   * @param[in] frame The desired frame.
   * @param[in] robot_state State from which the pose should be calculated.
   *
   * @return Vectorized 6x7 Jacobian, column-major.
   */
  std::array<double, 42> bodyJacobian(franka::Frame frame,
                                      const franka::RobotState& robot_state) const {
    return bodyJacobian(frame, robot_state.q, robot_state.F_T_EE, robot_state.EE_T_K);
  }

  /**
   * Gets the 6x7 Jacobian for the given frame, relative to that frame.
   *
   * The Jacobian is represented as a 6x7 matrix in column-major format.
   *
   * @param[in] frame The desired frame.
   * @param[in] q Joint position.
   * @param[in] F_T_EE End effector in flange frame.
   * @param[in] EE_T_K Stiffness frame K in the end effector frame.
   *
   * @return Vectorized 6x7 Jacobian, column-major.
   */
  virtual std::array<double, 42> bodyJacobian(
      franka::Frame frame,
      const std::array<double, 7>& q,
      const std::array<double, 16>& F_T_EE,  // NOLINT(readability-identifier-naming)
      const std::array<double, 16>& EE_T_K)  // NOLINT(readability-identifier-naming)
      const = 0;

  /**
   * Gets the 6x7 Jacobian for the given joint relative to the base frame.
   *
   * The Jacobian is represented as a 6x7 matrix in column-major format.
   *
   * @param[in] frame The desired frame.
   * @param[in] robot_state State from which the pose should be calculated.
   *
   * @return Vectorized 6x7 Jacobian, column-major.
   */
  std::array<double, 42> zeroJacobian(franka::Frame frame,
                                      const franka::RobotState& robot_state) const {
    return zeroJacobian(frame, robot_state.q, robot_state.F_T_EE, robot_state.EE_T_K);
  }

  /**
   * Gets the 6x7 Jacobian for the given joint relative to the base frame.
   *
   * The Jacobian is represented as a 6x7 matrix in column-major format.
   *
   * @param[in] frame The desired frame.
   * @param[in] q Joint position.
   * @param[in] F_T_EE End effector in flange frame.
   * @param[in] EE_T_K Stiffness frame K in the end effector frame.
   *
   * @return Vectorized 6x7 Jacobian, column-major.
   */
  virtual std::array<double, 42> zeroJacobian(
      franka::Frame frame,
      const std::array<double, 7>& q,
      const std::array<double, 16>& F_T_EE,  // NOLINT(readability-identifier-naming)
      const std::array<double, 16>& EE_T_K)  // NOLINT(readability-identifier-naming)
      const = 0;

  /**
   * Calculates the 7x7 mass matrix. Unit: \f$[kg \times m^2]\f$.
   *
   * @param[in] robot_state State from which the pose should be calculated.
   *
   * @return Vectorized 7x7 mass matrix, column-major.
   */
  std::array<double, 49> mass(const franka::RobotState& robot_state) const {
    return mass(robot_state.q, robot_state.I_total, robot_state.m_total, robot_state.F_x_Ctotal);
  }

  /**
   * Calculates the 7x7 mass matrix. Unit: \f$[kg \times m^2]\f$.
   *
   * @param[in] q Joint position.
   * @param[in] I_total Inertia of the attached total load including end effector, relative to
   * center of mass, given as vectorized 3x3 column-major matrix. Unit: \f$[kg \times m^2]\f$.
   * @param[in] m_total Weight of the attached total load including end effector.
   * Unit: \f$[kg]\f$.
   * @param[in] F_x_Ctotal Translation from flange to center of mass of the attached total load.
   * Unit: \f$[m]\f$.
   *
   * @return Vectorized 7x7 mass matrix, column-major.
   */
  virtual std::array<double, 49> mass(
      const std::array<double, 7>& q,
      const std::array<double, 9>& I_total,  // NOLINT(readability-identifier-naming)
      double m_total,
      const std::array<double, 3>& F_x_Ctotal)  // NOLINT(readability-identifier-naming)
      const = 0;

  /**
   * Calculates the Coriolis force vector (state-space equation): \f$ c= C \times
   * dq\f$, in \f$[Nm]\f$.
   *
   * @param[in] robot_state State from which the Coriolis force vector should be calculated.
   *
   * @return Coriolis force vector.
   */
  std::array<double, 7> coriolis(const franka::RobotState& robot_state) const {
    return coriolis(robot_state.q, robot_state.dq, robot_state.I_total, robot_state.m_total,
                    robot_state.F_x_Ctotal);
  }

  /**
   * Calculates the Coriolis force vector (state-space equation): \f$ c= C \times
   * dq\f$, in \f$[Nm]\f$.
   *
   * @param[in] q Joint position.
   * @param[in] dq Joint velocity.
   * @param[in] I_total Inertia of the attached total load including end effector, relative to
   * center of mass, given as vectorized 3x3 column-major matrix. Unit: \f$[kg \times m^2]\f$.
   * @param[in] m_total Weight of the attached total load including end effector.
   * Unit: \f$[kg]\f$.
   * @param[in] F_x_Ctotal Translation from flange to center of mass of the attached total load.
   * Unit: \f$[m]\f$.
   *
   * @return Coriolis force vector.
   */
  virtual std::array<double, 7> coriolis(
      const std::array<double, 7>& q,
      const std::array<double, 7>& dq,
      const std::array<double, 9>& I_total,  // NOLINT(readability-identifier-naming)
      double m_total,
      const std::array<double, 3>& F_x_Ctotal)  // NOLINT(readability-identifier-naming)
      const = 0;

  /**
   * Calculates the gravity vector. Unit: \f$[Nm]\f$. Assumes default gravity vector of -9.81 m/s^2
   *
   * @param[in] q Joint position.
   * @param[in] m_total Weight of the attached total load including end effector.
   * Unit: \f$[kg]\f$.
   * @param[in] F_x_Ctotal Translation from flange to center of mass of the attached total load.
   * Unit: \f$[m]\f$.
   *
   * @return Gravity vector.
   */
  std::array<double, 7> gravity(
      const std::array<double, 7>& q,
      double m_total,
      const std::array<double, 3>& F_x_Ctotal  // NOLINT(readability-identifier-naming)
      ) const {
    return gravity(q, m_total, F_x_Ctotal, {0, 0, -9.81});
  }

  /**
   * Calculates the gravity vector. Unit: \f$[Nm]\f$.
   *
   * @param[in] q Joint position.
   * @param[in] m_total Weight of the attached total load including end effector.
   * Unit: \f$[kg]\f$.
   * @param[in] F_x_Ctotal Translation from flange to center of mass of the attached total load.
   * Unit: \f$[m]\f$.
   * @param[in] gravity_earth Earth's gravity vector. Unit: \f$\frac{m}{s^2}\f$.
   *
   * @return Gravity vector.
   */
  virtual std::array<double, 7> gravity(
      const std::array<double, 7>& q,
      double m_total,
      const std::array<double, 3>& F_x_Ctotal,  // NOLINT(readability-identifier-naming)
      const std::array<double, 3>& gravity_earth) const = 0;

  /**
   * Calculates the gravity vector. Unit: \f$[Nm]\f$. Assumes default gravity vector of -9.81 m/s^2
   *
   * @param[in] robot_state State from which the gravity vector should be calculated.
   *
   * @return Gravity vector.
   */
  std::array<double, 7> gravity(const franka::RobotState& robot_state) const {
#ifdef ENABLE_BASE_ACCELERATION
    return gravity(robot_state.q, robot_state.m_total, robot_state.F_x_Ctotal, robot_state.O_ddP_O);
#else
    return gravity(robot_state.q, robot_state.m_total, robot_state.F_x_Ctotal, {0, 0, -9.81});
#endif
  }

  /**
   * Calculates the gravity vector. Unit: \f$[Nm]\f$.
   *
   * @param[in] robot_state State from which the gravity vector should be calculated.
   * @param[in] gravity_earth Earth's gravity vector. Unit: \f$\frac{m}{s^2}\f$.
   *
   * @return Gravity vector.
   */
  std::array<double, 7> gravity(const franka::RobotState& robot_state,
                                const std::array<double, 3>& gravity_earth) const {
    return gravity(robot_state.q, robot_state.m_total, robot_state.F_x_Ctotal, gravity_earth);
  }
};

}  // namespace franka_hw
