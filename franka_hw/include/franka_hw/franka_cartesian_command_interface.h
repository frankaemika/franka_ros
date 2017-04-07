#pragma once

#include <franka_hw/franka_cartesian_state_interface.h>
#include <hardware_interface/internal/hardware_resource_manager.h>
#include <string>

namespace franka_hw {

/** \brief A handle used to read and command a Cartesian Pose to a Franka. */
class FrankaCartesianPoseHandle : public FrankaCartesianStateHandle {
 public:
  FrankaCartesianPoseHandle() = delete;

  /**
   * \param cartesian_state_handle The cartesian state handle
   * \param command A reference to the storage field for the cartesian pose
   * passed a desired homogeneous transformation O_T_EE_d
   */
  FrankaCartesianPoseHandle(
      const FrankaCartesianStateHandle& cartesian_state_handle,
      std::array<double, 16>& command)
      : FrankaCartesianStateHandle(cartesian_state_handle),
        command_(&command) {}

  void setCommand(std::array<double, 16>& command) { *command_ = command; }
  const std::array<double, 16>& getCommand() const { return *command_; }

 private:
  std::array<double, 16>* command_;
};

/** \brief Hardware interface to support commanding an array of Franka joints.
 */
class FrankaPoseCartesianInterface
    : public hardware_interface::HardwareResourceManager<
          FrankaCartesianPoseHandle,
          hardware_interface::ClaimResources> {};

/** \brief A handle used to read and command a Cartesian Velocity to a Franka.
 */
class FrankaCartesianVelocityHandle : public FrankaCartesianStateHandle {
 public:
  FrankaCartesianVelocityHandle() = delete;

  /**
   * \param cartesian_state_handle The cartesian state handle
   * \param command A reference to the storage field for the cartesian pose
   * passed a desired homogeneous transformation O_T_EE_d
   */
  FrankaCartesianVelocityHandle(
      const FrankaCartesianStateHandle& cartesian_state_handle,
      std::array<double, 6>& command)
      : FrankaCartesianStateHandle(cartesian_state_handle),
        command_(&command) {}

  void setCommand(std::array<double, 6>& command) { *command_ = command; }
  const std::array<double, 6>& getCommand() const { return *command_; }

 private:
  std::array<double, 6>* command_;
};

/** \brief Hardware interface to support commanding an array of Franka joints.
 */
class FrankaVelocityCartesianInterface
    : public hardware_interface::HardwareResourceManager<
          FrankaCartesianVelocityHandle,
          hardware_interface::ClaimResources> {};

}  // namespace franka_hw
