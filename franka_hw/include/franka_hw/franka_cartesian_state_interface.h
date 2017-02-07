#ifndef HARDWARE_INTERFACE_FRANKA_CARTESIAN_STATE_INTERFACE_H
#define HARDWARE_INTERFACE_FRANKA_CARTESIAN_STATE_INTERFACE_H

#include <hardware_interface/internal/hardware_resource_manager.h>
#include <cassert>
#include <string>

namespace hardware_interface
{

/** A handle used to read the cartesian state of an end-effector. */
class FrankaCartesianStateHandle
{
public:
  FrankaCartesianStateHandle(): collision_(0), contact_(0), O_F_ext_hat_EE_(0), EE_F_ext_hat_EE_(0), O_T_EE_start_(0) {}

  /**
   * \param collision The collision state of the arm
   * \param contact The contact state of the arm
   * \param O_F_ext_hat_EE The external wrench exerted to the arm w.r.t. base_link coordinates
   * \param EE_F_ext_hat_EE The external wrench exerted to the arm w.r.t. end-effector coordinates
   * \param O_T_EE_start The homogeneous transformation matrix from end-effector to base_link frame
   */
  FrankaCartesianStateHandle(const std::vector<double>* collision,
                             const std::vector<double>* contact,
                             const std::vector<double>* O_F_ext_hat_EE,
                             const std::vector<double>* EE_F_ext_hat_EE,
                             const std::vector<std::vector<double> >* O_T_EE_start)
    : collision_(collision),
      contact_(contact),
      O_F_ext_hat_EE_(O_F_ext_hat_EE),
      EE_F_ext_hat_EE_(EE_F_ext_hat_EE),
      O_T_EE_start_(O_T_EE_start)
  {
    if (!collision)
    {
      throw HardwareInterfaceException("Cannot create handle for cartesian collision. Data pointer is null");
    }
    if (!contact)
    {
      throw HardwareInterfaceException("Cannot create handle for cartesian contact. Data pointer is null");
    }
    if (!O_F_ext_hat_EE)
    {
      throw HardwareInterfaceException("Cannot create handle for O_F_ext_hat_EE external wrench. Data pointer is null");
    }
    if (!EE_F_ext_hat_EE)
    {
      throw HardwareInterfaceException("Cannot create handle for EE_F_ext_hat_EE external wrench. Data pointer is null");
    }
    if (!O_T_EE_start)
    {
      throw HardwareInterfaceException("Cannot create handle for O_T_EE_start end-effector transform."
                                       " Data pointer is null");
    }
  }

  std::vector<double> getCollision()  const {assert(collision_); return *collision_;}
  std::vector<double> getContact()  const {assert(contact_); return *contact_;}
  std::vector<double> getFExtO()  const {assert(O_F_ext_hat_EE_); return *O_F_ext_hat_EE_;}
  std::vector<double> getFExtEE()  const {assert(EE_F_ext_hat_EE_); return *EE_F_ext_hat_EE_;}
  std::vector<std::vector<double> > getTransform()  const {assert(O_T_EE_start_); return *O_T_EE_start_;}

private:
  const std::vector<double>* collision_;
  const std::vector<double>* contact_;
  const std::vector<double>* O_F_ext_hat_EE_;
  const std::vector<double>* EE_F_ext_hat_EE_;
  const std::vector<std::vector<double> >* O_T_EE_start_;
};

/** \brief Hardware interface to support reading the cartesian state of a franka end-effector
 *
 * This \ref HardwareInterface supports reading the cartesian state of an end-effector attached to a franka emika
 * arm. This inludes a collision state, a contact state, estimated external wrench exerted to the robot w.r.t. the end-
 * effector frame and the robot base_link and the homogenous transformation from End-effector frame to base_link frame
 */
class FrankaCartesianStateInterface : public HardwareResourceManager<FrankaCartesianStateHandle> {};

}  // namespace hardware_interface

#endif  // HARDWARE_INTERFACE_FRANKA_CARTESIAN_STATE_INTERFACE_H
