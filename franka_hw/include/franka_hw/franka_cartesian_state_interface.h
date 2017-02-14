#pragma once

#include <hardware_interface/internal/hardware_resource_manager.h>
#include <array>
#include <cassert>
#include <string>

namespace hardware_interface {

/** A handle used to read the cartesian state of an end-effector. */
class FrankaCartesianStateHandle {
public:
    /**
   * \param collision The collision state of the arm
   * \param contact The contact state of the arm
   * \param O_F_ext_hat_EE The external wrench exerted to the arm w.r.t.
   * base_link coordinates
   * \param EE_F_ext_hat_EE The external wrench exerted to the arm w.r.t.
   * end-effector coordinates
   * \param O_T_EE_start The homogeneous transformation matrix from end-effector
   * to base_link frame
   */
    FrankaCartesianStateHandle(
            const std::string &name,
            const std::array<double, 6> &collision,
            const std::array<double, 6> &contact,
            const std::array<double, 6> &O_F_ext_hat_EE,
            const std::array<double, 6> &EE_F_ext_hat_EE,
            const std::array<double, 16> &O_T_EE_start)
        : name_(name),
          collision_(&collision),
          contact_(&contact),
          O_F_ext_hat_EE_(&O_F_ext_hat_EE),
          EE_F_ext_hat_EE_(&EE_F_ext_hat_EE),
          O_T_EE_start_(&O_T_EE_start) { }

    std::string getName() const { return name_; }
    std::array<double, 6> getCollision() const {
        assert(collision_);
        return *collision_;
    }
    std::array<double, 6> getContact() const {
        assert(contact_);
        return *contact_;
    }
    std::array<double, 6> getFExtO() const {
        assert(O_F_ext_hat_EE_);
        return *O_F_ext_hat_EE_;
    }
    std::array<double, 6> getFExtEE() const {
        assert(EE_F_ext_hat_EE_);
        return *EE_F_ext_hat_EE_;
    }
    std::array<double, 16> getTransform() const {
        assert(O_T_EE_start_);
        return *O_T_EE_start_;
    }

private:
    std::string name_;
    const std::array<double, 6> *collision_;
    const std::array<double, 6> *contact_;
    const std::array<double, 6> *O_F_ext_hat_EE_;
    const std::array<double, 6> *EE_F_ext_hat_EE_;
    const std::array<double, 16> *O_T_EE_start_;
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
        : public HardwareResourceManager<FrankaCartesianStateHandle> {};

}  // namespace hardware_interface
