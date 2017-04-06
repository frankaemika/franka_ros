# pragma once

#include <string>
#include <hardware_interface/internal/hardware_resource_manager.h>
#include <franka_hw/franka_joint_state_interface.h>

namespace franka_hw
{

/** \brief A handle used to read and command a single joint. */
class FrankaJointHandle : public FrankaJointStateHandle
{
public:
  FrankaJointHandle() = delete;

  /**
   * \param joint_state_handle This joint's state handle
   * \param command A reference to the storage field for the joint command
   */
  FrankaJointHandle(const FrankaJointStateHandle& joint_state_handle, double& command)
    : FrankaJointStateHandle(joint_state_handle),
      command_(&command) {}

  void setCommand(double command) {*command_ = command; }
  const double& getCommand() const { return *command_; }

private:
  double* command_;
};

/** \brief Hardware interface to support commanding an array of Franka joints. */
class FrankaJointCommandInterface : public hardware_interface::HardwareResourceManager<
    FrankaJointHandle,
    hardware_interface::ClaimResources> {};

/// \brief JointCommandInterface for commanding joint torques.
class FrankaEffortJointInterface : public FrankaJointCommandInterface {};

/// \brief JointCommandInterface for commanding joint velocities.
class FrankaVelocityJointInterface : public FrankaJointCommandInterface {};

/// \brief JointCommandInterface for commanding joint positions.
class FrankaPositionJointInterface : public FrankaJointCommandInterface {};

}  // namespace franka_hw
