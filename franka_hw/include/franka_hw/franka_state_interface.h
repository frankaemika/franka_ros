#pragma once

#include <franka/robot_state.h>
#include <hardware_interface/internal/hardware_resource_manager.h>
#include <array>
#include <cassert>
#include <string>

namespace franka_hw {

/** A handle used to read the cartesian state of an end-effector. */
class FrankaStateHandle {
 public:
  FrankaStateHandle() = delete;
  /**
 * \param name The name of the state handle
 * \param robot_state A reference to the robot_state
 */
  FrankaStateHandle(const std::string& name, franka::RobotState& robot_state)
      : name_(name), robot_state_(&robot_state) {}

  const std::string& getName() const { return name_; }
  const franka::RobotState getRobotState() const { return *robot_state_; }

 private:
  std::string name_;
  const franka::RobotState* robot_state_;
};

/** \brief Hardware interface to support reading the complete robot state of a
 * franka
 *  robot
 *
 * This \ref HardwareInterface supports reading the state of a franka robot
 * This inludes a collision state, a contact state, estimated external
 * wrench exerted to the robot w.r.t. the end-
 * effector frame and the robot base_link and the homogenous transformation from
 * End-effector frame to base_link frame as well joint_specific states
 */
class FrankaStateInterface
    : public hardware_interface::HardwareResourceManager<FrankaStateHandle> {};

}  // namespace franka_hw
