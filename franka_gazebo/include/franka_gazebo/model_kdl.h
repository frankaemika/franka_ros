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
 * Calculates poses of links and dynamic properties of the robot.
 *
 * This implementation of @ref ModelBase uses KDL as backend for calculating
 * dynamic and kinematic properties of the robot.
 */
class ModelKDL : public franka_hw::ModelBase {
 public:
  /**
   * Create a new implementation for the \ref ModelBase with KDL as backend
   *
   * @param[in] model the URDF from which to interprete the kinematic chain
   * @param[in] root the link name of the root of the chain
   * @param[in] tip the link name of the tip of the chain
   * @param[in] singularity_threshold below which lowest singular value of SVD(J x J^T)
   *            a warning should be printed. Use -1 to disable.
   *
   * @throws std::invalid_argument when either `root` or `tip` cannot be found in the URDF
   */
  ModelKDL(const urdf::Model& model,
           const std::string& root,
           const std::string& tip,
           double singularity_threshold = -1);

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
  std::array<double, 16> pose(
      franka::Frame frame,
      const std::array<double, 7>& q,
      const std::array<double, 16>& F_T_EE,  // NOLINT(readability-identifier-naming)
      const std::array<double, 16>& EE_T_K)  // NOLINT(readability-identifier-naming)
      const override;

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
   *
   * @note Not yet implemented
   */
  std::array<double, 42> bodyJacobian(
      franka::Frame frame,
      const std::array<double, 7>& q,
      const std::array<double, 16>& F_T_EE,  // NOLINT(readability-identifier-naming)
      const std::array<double, 16>& EE_T_K)  // NOLINT(readability-identifier-naming)
      const override;

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
  std::array<double, 42> zeroJacobian(
      franka::Frame frame,
      const std::array<double, 7>& q,
      const std::array<double, 16>& F_T_EE,  // NOLINT(readability-identifier-naming)
      const std::array<double, 16>& EE_T_K)  // NOLINT(readability-identifier-naming)
      const override;

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
  std::array<double, 49> mass(
      const std::array<double, 7>& q,
      const std::array<double, 9>& I_total,  // NOLINT(readability-identifier-naming)
      double m_total,
      const std::array<double, 3>& F_x_Ctotal)  // NOLINT(readability-identifier-naming)
      const override;

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
  std::array<double, 7> coriolis(
      const std::array<double, 7>& q,
      const std::array<double, 7>& dq,
      const std::array<double, 9>& I_total,  // NOLINT(readability-identifier-naming)
      double m_total,
      const std::array<double, 3>& F_x_Ctotal)  // NOLINT(readability-identifier-naming)
      const override;

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
  std::array<double, 7> gravity(
      const std::array<double, 7>& q,
      double m_total,
      const std::array<double, 3>& F_x_Ctotal,  // NOLINT(readability-identifier-naming)
      const std::array<double, 3>& gravity_earth) const override;

  /**
   * Augment a kinematic chain by adding a virtual frame to it.
   *
   * @param[in] name the name of the frame. Must be unique in the `chain`
   * @param[in] transform the homogenous transformation of the frame relative to the tip of `chain`
   * which to add to the chain
   * @param[out] chain a reference to the kinematic chain which to augment. The segment will be
   * added to this chain, so if you want to keep your original chain, make a copy before
   */
  static void augmentFrame(const std::string& name,
                           const std::array<double, 16>& transform,
                           KDL::Chain& chain);

  /**
   * Augment a kinematic chain by adding a virtual frame with an inertia to it.
   *
   * @param[in] name the name of the frame. Must be unique in the `chain`
   * @param[in] center_of_mass the position of the center of mass of the virtual frame which should
   * be added, relative to the tip of `chain`
   * @param[in] mass the mass in \f$[kg]\f$ of this virtual frame
   * @param[in] inertia the 3d inertia tensor (column major) of the new frame, measured at
   * `center_of_mass`.
   * @param[out] chain a reference to the kinematic chain which to augment. The segment will be
   * added to this chain, so if you want to keep your original chain, make a copy before
   */
  static void augmentFrame(const std::string& name,
                           const std::array<double, 3>& center_of_mass,
                           double mass,
                           const std::array<double, 9>& inertia,
                           KDL::Chain& chain);

 private:
  static int segment(franka::Frame frame);
  static std::string strError(const int error);
  bool isCloseToSingularity(const KDL::Jacobian& jacobian) const;

  KDL::Chain chain_;
  double singularity_threshold_;
};

}  // namespace franka_gazebo
