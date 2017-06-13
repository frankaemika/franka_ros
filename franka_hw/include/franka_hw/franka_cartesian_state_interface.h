#pragma once

#include <hardware_interface/internal/hardware_resource_manager.h>
#include <array>
#include <cassert>
#include <string>

namespace franka_hw {

/** A handle used to read the cartesian state of an end-effector. */
class FrankaCartesianStateHandle {
 public:
  FrankaCartesianStateHandle() = delete;

  /**
 * \param collision The collision state of the arm
 * \param contact The contact state of the arm
 * \param O_F_ext_hat_K The external wrench exerted to the arm w.r.t.
 * base_link coordinates
 * \param K_F_ext_hat_K The external wrench exerted to the arm w.r.t.
 * end-effector coordinates
 * \param O_T_EE The homogeneous transformation matrix from end-effector
 * to base_link frame
 */
  FrankaCartesianStateHandle(const std::string& name,
                             const std::array<double, 6>& collision,
                             const std::array<double, 6>& contact,
                             const std::array<double, 6>& O_F_ext_hat_K,
                             const std::array<double, 6>& K_F_ext_hat_K,
                             const std::array<double, 16>& O_T_EE,
                             const std::array<double, 16>& O_T_EE_d)
      : name_(name),
        collision_(&collision),
        contact_(&contact),
        O_F_ext_hat_K_(&O_F_ext_hat_K),
        K_F_ext_hat_K_(&K_F_ext_hat_K),
        O_T_EE_(&O_T_EE),
        O_T_EE_d_(&O_T_EE_d) {}

  const std::string& getName() const { return name_; }
  const std::array<double, 6>& getCollision() const { return *collision_; }
  const std::array<double, 6>& getContact() const { return *contact_; }
  const std::array<double, 6>& getFExtO() const { return *O_F_ext_hat_K_; }
  const std::array<double, 6>& getFExtK() const { return *K_F_ext_hat_K_; }
  const std::array<double, 16>& getTransform() const { return *O_T_EE_; }
  const std::array<double, 16>& getDesiredTransform() const { return *O_T_EE_d_; }

 private:
  std::string name_;
  const std::array<double, 6>* collision_;
  const std::array<double, 6>* contact_;
  const std::array<double, 6>* O_F_ext_hat_K_;
  const std::array<double, 6>* K_F_ext_hat_K_;
  const std::array<double, 16>* O_T_EE_;
  const std::array<double, 16>* O_T_EE_d_;
};

/** \brief Hardware interface to support reading the cartesian state of a franka
 * end-effector
 *
 * This \ref HardwareInterface supports reading the cartesian state of an
 * end-effector attached to a franka emika
 * arm. This inludes a collision state, a contact state, estimated external
 * wrench exerted to the robot w.r.t. the end-
 * effector frame and the robot base_link and the homogenous transformation from
 * End-effector frame to base_link frame
 */
class FrankaCartesianStateInterface
    : public hardware_interface::HardwareResourceManager<
          FrankaCartesianStateHandle> {};

}  // namespace franka_hw
