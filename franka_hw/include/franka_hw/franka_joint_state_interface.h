#ifndef FRANKA_HW_FRANKA_JOINT_STATE_INTERFACE_H
#define FRANKA_HW_FRANKA_JOINT_STATE_INTERFACE_H

#include <hardware_interface/internal/hardware_resource_manager.h>
#include <cassert>
#include <string>

namespace hardware_interface
{

/** A handle used to read the state of a single Franka joint. */
class FrankaJointStateHandle
{
public:
    FrankaJointStateHandle() : name_(), q_(0), dq_(0), tau_J_(0), q_d_(0),
        q_start_(0), dtau_J_(0), tau_ext_hat_filtered_(0),
        joint_collision_(0), joint_contact_(0) {}

   /**
   * \param name_ The name of the joint
   * \param q A pointer to the storage for this joint's position
   * \param dq A pointer to the storage for this joint's velocity
   * \param tau_J A pointer to the storage for this joint's torque
   * \param q_d A pointer to the storage for this joint's desired position
   * \param q_start A pointer to the storage for this joint's TODO
   * \param dtau_J A pointer to the storage for this joint's torque's time derivative
   * \param tau_ext_hat_filtered A pointer to the storage for this joint's TODO
   * \param joint_collision A pointer to the storage for this joint's TODO
   * \param joint_contact A pointer to the storage for this joint's TODO
   *
   */
    FrankaJointStateHandle(const std::string& name, const double* q, const double* dq, const double* tau_J,
                           const double* q_d, const double* q_start, const double* dtau_J,
                           const double* tau_ext_hat_filtered, const double* joint_collision,
                           const double* joint_contact)
        : name_(name), q_(q), dq_(dq), tau_J_(tau_J), q_d_(q_d), q_start_(q_start), dtau_J_(dtau_J),
          tau_ext_hat_filtered_(tau_ext_hat_filtered), joint_collision_(joint_collision), joint_contact_(joint_contact)
    {
        if (!q)
        {
            throw HardwareInterfaceException("Cannot create handle '" + name + "'. q data pointer is null.");
        }
        if (!dq)
        {
            throw HardwareInterfaceException("Cannot create handle '" + name + "'. dq data pointer is null.");
        }
        if (!tau_J)
        {
            throw HardwareInterfaceException("Cannot create handle '" + name + "'. tau_J data pointer is null.");
        }
        if (!q_d)
        {
            throw HardwareInterfaceException("Cannot create handle '" + name + "'. q_d data pointer is null.");
        }
        if (!q_start)
        {
            throw HardwareInterfaceException("Cannot create handle '" + name + "'. q_start data pointer is null.");
        }
        if (!dtau_J)
        {
            throw HardwareInterfaceException("Cannot create handle '" + name + "'. dtau_J data pointer is null.");
        }
        if (!tau_ext_hat_filtered)
        {
            throw HardwareInterfaceException("Cannot create handle '" + name + "'. tau_ext_hat_filtered data pointer is null.");
        }
        if (!joint_collision)
        {
            throw HardwareInterfaceException("Cannot create handle '" + name + "'. joint_collision data pointer is null.");
        }
        if (!joint_contact)
        {
            throw HardwareInterfaceException("Cannot create handle '" + name + "'. joint_contact data pointer is null.");
        }
    }

    std::string getName() const {return name_;}
    double getPosition()  const {assert(q_); return *q_;}
    double getVelocity()  const {assert(dq_); return *dq_;}
    double getEffort()    const {assert(tau_J_); return *tau_J_;}
    double getQd()  const {assert(q_d_); return *q_d_;}
    double getQstart()  const {assert(q_start_); return *q_start_;}
    double getDtauj()  const {assert(dtau_J_); return *dtau_J_;}
    double getTauExtHatFiltered()  const {assert(tau_ext_hat_filtered_); return *tau_ext_hat_filtered_;}
    double getJointCollision()  const {assert(joint_collision_); return *joint_collision_;}
    double getJointContact()  const {assert(joint_contact_); return *joint_contact_;}

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

/** \brief Hardware interface to support reading the state of an array of Franka joints
 *
 * This \ref HardwareInterface supports reading the state of an array of named
 * Franka joints, each of which has some position, velocity, and effort and additional quantities (e.g. collision and
 * contact states etc.).
 *
 */
class FrankaJointStateInterface : public HardwareResourceManager<FrankaJointStateHandle> {};

}  // namespace hardware_interface

#endif  // FRANKA_HW_FRANKA_JOINT_STATE_INTERFACE_H
