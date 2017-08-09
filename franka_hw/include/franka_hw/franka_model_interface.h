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
   * Returns the Cartesian pose of a joint w.r.t. the O-frame of the robot
   *
   * @param[in] joint_name The name of the joint
   * @param[in] robot_state A user-given robot state to evaluate the forward kinematics
   *
   * @return Vectorized 4x4 homogeneous transform, column-major.
   */
  std::array<double, 16> getJointPose(const std::string& joint_name,
                                      const franka::RobotState& robot_state) {
    franka::Frame frame;
    if (getFrameFromName(joint_name, &frame)) {
      return model_->jointPose(frame, robot_state);
    }
    return {{1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0}};
  }

  /**
   * Returns the Cartesian pose of a joint w.r.t. the O-frame of the robot
   *
   * @param[in] joint_name The name of the joint
   *
   * @return Vectorized 4x4 homogeneous transform, column-major.
   */
  std::array<double, 16> getJointPose(const std::string& joint_name) {
    return getJointPose(joint_name, *robot_state_);
  }

  /**
   * Gets the 6x7 Jacobian for the given joint w.r.t. the joint frame. The Jacobian is represented
   * as a 6x7 matrix in column-major format and is evaluated at the given robot state
   *
   * @param[in] joint_name The name of the desired joint.
   * @param[in] robot_state State from which the pose should be calculated.
   *
   * @return Vectorized 6x7 Jacobian, column-major.
   */
  std::array<double, 42> getBodyJacobian(const std::string& joint_name,
                                         const franka::RobotState& robot_state) {
    franka::Frame frame;
    if (getFrameFromName(joint_name, &frame)) {
      return model_->bodyJacobian(frame, robot_state);
    }
    ROS_WARN_STREAM("FrankaModelInterface: Could not get Jacobian with name " << joint_name);
    std::array<double, 42> empty{};
    return empty;
  }

  /**
   * Gets the 6x7 Jacobian for the given joint w.r.t. the joint frame and for the current robot
   * state. The Jacobian is represented as a 6x7 matrix in column-major format.
   *
   * @param[in] joint_name The name of the desired joint.
   *
   * @return Vectorized 6x7 Jacobian, column-major.
   */
  std::array<double, 42> getBodyJacobian(const std::string& joint_name) {
    return getBodyJacobian(joint_name, *robot_state_);
  }

  /**
   * Gets the 6x7 Jacobian for the given joint w.r.t. the link0 frame. The Jacobian is represented
   * as a 6x7 matrix in column-major format and is evaluated at the given robot state
   *
   * @param[in] joint_name The name of the desired joint.
   * @param[in] robot_state State from which the pose should be calculated.
   *
   * @return Vectorized 6x7 Jacobian, column-major.
   */
  std::array<double, 42> getZeroJacobian(const std::string& joint_name,
                                         const franka::RobotState& robot_state) {
    franka::Frame frame;
    if (getFrameFromName(joint_name, &frame)) {
      return model_->zeroJacobian(frame, robot_state);
    }
    ROS_WARN_STREAM("FrankaModelInterface: Could not get Jacobian with name " << joint_name);
    std::array<double, 42> empty{};
    return empty;
  }

  /**
   * Gets the 6x7 Jacobian for the given joint w.r.t. the link0 frame and for the current robot
   * state. The Jacobian is represented as a 6x7 matrix in column-major format.
   *
   * @param[in] joint_name The name of the desired joint.
   *
   * @return Vectorized 6x7 Jacobian, column-major.
   */
  std::array<double, 42> getZeroJacobian(const std::string& joint_name) {
    return getZeroJacobian(joint_name, *robot_state_);
  }

 private:
  bool getFrameFromName(const std::string& name, franka::Frame* frame) {
    int joint_index = std::atoi(&name.back());
    std::string joint_string("joint");
    joint_string.push_back(name.back());
    if (joint_index >= 1 && joint_index <= 7 && name.rfind(joint_string) == name.length() - 6 &&
        name.rfind(joint_string) != std::string::npos) {
      switch (joint_index) {
        case 1: {
          *frame = franka::Frame::kJoint1;
        }
        case 2: {
          *frame = franka::Frame::kJoint2;
        }
        case 3: {
          *frame = franka::Frame::kJoint3;
        }
        case 4: {
          *frame = franka::Frame::kJoint4;
        }
        case 5: {
          *frame = franka::Frame::kJoint5;
        }
        case 6: {
          *frame = franka::Frame::kJoint6;
        }
        case 7: {
          *frame = franka::Frame::kJoint7;
        }
      }
      return true;
    }
    size_t position_ee = name.rfind("_EE");
    if (position_ee == name.length() - 3 && position_ee != std::string::npos) {
      *frame = franka::Frame::kEndEffector;
      return true;
    }
    size_t position_f = name.rfind("_F");
    size_t position_link8 = name.rfind("link8");
    if (position_f == name.length() - 2 && position_f != std::string::npos ||
        position_link8 != std::string::npos) {
      *frame = franka::Frame::kFlange;
      return true;
    }
    return false;
  }

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
