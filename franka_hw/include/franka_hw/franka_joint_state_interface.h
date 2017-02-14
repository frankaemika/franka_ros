#pragma once

#include <hardware_interface/internal/hardware_resource_manager.h>
#include <cassert>
#include <string>

namespace hardware_interface {

/** A handle used to read the state of a single Franka joint. */
class FrankaJointStateHandle {
 public:
  /**
* \param name The name of the joint
* \param q A pointer to the storage for this joint's position
* \param dq A pointer to the storage for this joint's velocity
* \param tauJ A pointer to the storage for this joint's torque
* \param q_d A pointer to the storage for this joint's desired position
* \param q_start A pointer to the storage for this joint's interpolator start
* pose
* \param dtau_J A pointer to the storage for this joint's torque's time
* derivative
* \param tau_ext_hat_filtered A pointer to the storage for this joint's external
* torque
* \param joint_collision A pointer to the storage for this joint's collision
* state
* \param joint_contact A pointer to the storage for this joint's contact state
*/
  FrankaJointStateHandle(const std::string& name,
                         const double& q,
                         const double& dq,
                         const double& tau_J,
                         const double& q_d,
                         const double& q_start,
                         const double& dtau_J,
                         const double& tau_ext_hat_filtered,
                         const double& joint_collision,
                         const double& joint_contact)
      : name_(name),
        q_(&q),
        dq_(&dq),
        tau_J_(&tau_J),
        q_d_(&q_d),
        q_start_(&q_start),
        dtau_J_(&dtau_J),
        tau_ext_hat_filtered_(&tau_ext_hat_filtered),
        joint_collision_(&joint_collision),
        joint_contact_(&joint_contact) {}

  std::string getName() const { return name_; }
  const double& getPosition() const {
    return *q_;
  }
  const double& getVelocity() const {
    return *dq_;
  }
  const double& getEffort() const {
    return *tau_J_;
  }
  const double& getQd() const {
    return *q_d_;
  }
  const double& getQstart() const {
    return *q_start_;
  }
  const double& getDtauj() const {
    return *dtau_J_;
  }
  const double& getTauExtHatFiltered() const {
    return *tau_ext_hat_filtered_;
  }
  const double& getJointCollision() const {
    return *joint_collision_;
  }
  const double& getJointContact() const {
    return *joint_contact_;
  }

 private:
  std::string name_;
  const double* q_;
  const double* dq_;
  const double* tau_J_;
  const double* q_d_;
  const double* q_start_;
  const double* dtau_J_;
  const double* tau_ext_hat_filtered_;
  const double* joint_collision_;
  const double* joint_contact_;
};

/** \brief Hardware interface to support reading the state of an array of Franka
 * joints
 *
 * This \ref HardwareInterface supports reading the state of an array of named
 * Franka joints, each of which has some position, velocity, and effort and
 * additional quantities (e.g. collision and
 * contact states etc.).
 *
 */
class FrankaJointStateInterface
    : public HardwareResourceManager<FrankaJointStateHandle> {};

}  // namespace hardware_interface
