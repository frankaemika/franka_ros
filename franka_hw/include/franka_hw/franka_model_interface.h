#pragma once

#include <stdlib.h>
#include <array>
#include <string>

#include <franka/model.h>
#include <franka/robot_state.h>
#include <hardware_interface/internal/hardware_resource_manager.h>

namespace franka_hw {

/** A handle used to get the dynamic model of franka in joint-space. */
class FrankaModelHandle {
 public:
  FrankaModelHandle() = delete;

  /**
   * Constructs an instance of a FrankaModelHandle
   *
   * @param[in] name The name of the handle
   * @param[in] model A reference to a a franka::Model model instance
   * @param[in] robot_state A reference to the storage of the current robot state
   */
  FrankaModelHandle(const std::string& name, franka::Model& model, franka::RobotState& robot_state)
      : name_(name), model_(&model), robot_state_(&robot_state) {}

  /**
   * Returns the resource name of the Handle
   */
  std::string getName() const { return name_; }

  /**
   * Returns the inertia matrix, given the current robot state and given external
   * loads
   *
   * @param[in] load_inertia The column major inertia tensor for a load w.r.t. F
   * frame
   * @param[in] load_mass The mass of the load
   * @param[in] F_x_Cload The center of mass of the load w.r.t. F-frame
   */
  std::array<double, 49> getMass(
      const std::array<double, 9>& load_inertia,
      double load_mass,
      const std::array<double, 3>& F_x_Cload) const {  // NOLINT (readability-identifier-naming)
    return model_->mass(*robot_state_, load_inertia, load_mass, F_x_Cload);
  }

  /**
   * Returns the inertia matrix, given the input robot state and given external
   * loads
   *
   * @param[in] robot_state A user-given robot state to evaluate the dynamics
   * @param[in] load_inertia The column major inertia tensor for a load w.r.t. F
   * frame
   * @param[in] load_mass The mass of the load
   * @param[in] F_x_Cload The center of mass of the load w.r.t. F-frame
   */
  std::array<double, 49> getMass(
      const franka::RobotState& robot_state,
      const std::array<double, 9>& load_inertia,
      double load_mass,
      const std::array<double, 3>& F_x_Cload) const {  // NOLINT (readability-identifier-naming)
    return model_->mass(robot_state, load_inertia, load_mass, F_x_Cload);
  }

  /**
   * Returns the coriolis torques given the current robot state and given
   * external loads
   *
   * @param[in] load_inertia The column major inertia tensor for a load w.r.t. F
   * frame
   * @param[in] load_mass The mass of the load
   * @param[in] F_x_Cload The center of mass of the load w.r.t. F-frame
   */
  std::array<double, 7> getCoriolis(
      const std::array<double, 9>& load_inertia,
      double load_mass,
      const std::array<double, 3>& F_x_Cload) const {  // NOLINT (readability-identifier-naming)
    return model_->coriolis(*robot_state_, load_inertia, load_mass, F_x_Cload);
  }

  /**
   * Returns the coriolis torques, given the input robot state and given external
   * loads
   *
   * @param[in] robot_state A user-given robot state to evaluate the dynamics
   * @param[in] load_inertia The column major inertia tensor for a load w.r.t. F
   * frame
   * @param[in] load_mass The mass of the load
   * @param[in] F_x_Cload The center of mass of the load w.r.t. F-frame
   */
  std::array<double, 7> getCoriolis(
      const franka::RobotState& robot_state,
      const std::array<double, 9>& load_inertia,
      double load_mass,
      const std::array<double, 3>& F_x_Cload) const {  // NOLINT (readability-identifier-naming)
    return model_->coriolis(robot_state, load_inertia, load_mass, F_x_Cload);
  }

  /**
   * Returns the gravity torques, given the current robot state and given
   * external loads
   *
   * @param[in] load_inertia The column major inertia tensor for a load w.r.t. F
   * frame
   * @param[in] load_mass The mass of the load
   * @param[in] F_x_Cload The center of mass of the load w.r.t. F-frame
   */
  std::array<double, 7> getGravity(
      double load_mass,
      const std::array<double, 3>& F_x_Cload,  // NOLINT (readability-identifier-naming)
      const std::array<double, 3>& gravity_earth = {{0., 0., -9.81}}) const {
    return model_->gravity(*robot_state_, load_mass, F_x_Cload, gravity_earth);
  }

  /**
   * Returns the gravity torques, given the input robot state and given external
   * loads
   *
   * @param[in] robot_state A user-given robot state to evaluate the dynamics
   * @param[in] load_inertia The column major inertia tensor for a load w.r.t. F
   * frame
   * @param[in] load_mass The mass of the load
   * @param[in] F_x_Cload The center of mass of the load w.r.t. F-frame
   */
  std::array<double, 7> getGravity(
      const franka::RobotState& robot_state,
      double load_mass,
      const std::array<double, 3>& F_x_Cload,  // NOLINT (readability-identifier-naming)
      const std::array<double, 3>& gravity_earth = {{0., 0., -9.81}}) const {
    return model_->gravity(robot_state, load_mass, F_x_Cload, gravity_earth);
  }

  /**
   * Returns the Cartesian pose matrix of a frame w.r.t. the O-frame of the robot
   *
   * @param[in] frame The desired frame.
   * @param[in] robot_state A user-given robot state to evaluate the forward kinematics.
   *
   * @return Vectorized 4x4 homogeneous transform, column-major.
   */
  std::array<double, 16> getPose(const franka::Frame& frame,
                                 const franka::RobotState& robot_state) {
    return model_->pose(frame, robot_state);
  }

  /**
   * Returns the Cartesian pose matrix of a frame w.r.t. the O-frame of the robot
   *
   * @param[in] frame The desired frame.
   *
   * @return Vectorized 4x4 homogeneous transform, column-major.
   */
  std::array<double, 16> getPose(const franka::Frame& frame) {
    return getPose(frame, *robot_state_);
  }

  /**
   * Gets the 6x7 Jacobian for the given frame w.r.t. that frame. The Jacobian is represented
   * as a 6x7 matrix in column-major format and is evaluated at the given robot state
   *
   * @param[in] frame The desired frame.
   * @param[in] robot_state State from which the pose should be calculated.
   *
   * @return Vectorized 6x7 Jacobian, column-major.
   */
  std::array<double, 42> getBodyJacobian(const franka::Frame& frame,
                                         const franka::RobotState& robot_state) {
    return model_->bodyJacobian(frame, robot_state);
  }

  /**
   * Gets the 6x7 Jacobian for the given frame w.r.t. that frame and for the current robot
   * state. The Jacobian is represented as a 6x7 matrix in column-major format.
   *
   * @param[in] frame The desired frame.
   *
   * @return Vectorized 6x7 Jacobian, column-major.
   */
  std::array<double, 42> getBodyJacobian(const franka::Frame& frame) {
    return getBodyJacobian(frame, *robot_state_);
  }

  /**
   * Gets the 6x7 Jacobian for the given frame w.r.t. the link0 frame. The Jacobian is represented
   * as a 6x7 matrix in column-major format and is evaluated at the given robot state
   *
   * @param[in] frame The desired frame.
   * @param[in] robot_state State from which the pose should be calculated.
   *
   * @return Vectorized 6x7 Jacobian, column-major.
   */
  std::array<double, 42> getZeroJacobian(const franka::Frame& frame,
                                         const franka::RobotState& robot_state) {
    return model_->zeroJacobian(frame, robot_state);
  }

  /**
   * Gets the 6x7 Jacobian for the given frame w.r.t. the link0 frame and for the current robot
   * state. The Jacobian is represented as a 6x7 matrix in column-major format.
   *
   * @param[in] frame The desired frame.
   *
   * @return Vectorized 6x7 Jacobian, column-major.
   */
  std::array<double, 42> getZeroJacobian(const franka::Frame& frame) {
    return getZeroJacobian(frame, *robot_state_);
  }

 private:
  std::string name_;
  const franka::Model* model_;
  const franka::RobotState* robot_state_;
};

/** \brief Hardware interface to support reading the joint-space dynamics of a
 * franka robot
 *
 * This \ref HardwareInterface supports reading the state of an array of named
 * Franka joints, each of which has some position, velocity, and effort and
 * additional quantities (e.g. collision and
 * contact states etc.).
 *
 */
class FrankaModelInterface : public hardware_interface::HardwareResourceManager<FrankaModelHandle> {
};

}  // namespace franka_hw
