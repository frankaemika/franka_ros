#pragma once

#include <map>
#include <ostream>
#include <string>
#include <vector>

namespace franka_hw {

using ResourceWithClaimsMap =
    std::map<std::string, std::vector<std::vector<std::string>>>;

struct ResourceClaims {
  uint8_t joint_position_claims = 0;
  uint8_t joint_velocity_claims = 0;
  uint8_t joint_torque_claims = 0;
  uint8_t cartesian_velocity_claims = 0;
  uint8_t cartesian_pose_claims = 0;
};

using ArmClaimedMap = std::map<std::string, ResourceClaims>;

enum class ControlMode {
  None = 0,
  JointTorque = (1 << 0),
  JointPosition = (1 << 1),
  JointVelocity = (1 << 2),
  CartesianVelocity = (1 << 3),
  CartesianPose = (1 << 4),
};

std::ostream& operator<<(std::ostream& ostream, ControlMode mode);

// Implement operators for BitmaskType concept
constexpr ControlMode operator&(ControlMode left, ControlMode right) {
  return static_cast<ControlMode>(
      static_cast<std::underlying_type_t<ControlMode>>(left) &
      static_cast<std::underlying_type_t<ControlMode>>(right));
}

constexpr ControlMode operator|(ControlMode left, ControlMode right) {
  return static_cast<ControlMode>(
      static_cast<std::underlying_type_t<ControlMode>>(left) |
      static_cast<std::underlying_type_t<ControlMode>>(right));
}

constexpr ControlMode operator^(ControlMode left, ControlMode right) {
  return static_cast<ControlMode>(
      static_cast<std::underlying_type_t<ControlMode>>(left) ^
      static_cast<std::underlying_type_t<ControlMode>>(right));
}

constexpr ControlMode operator~(ControlMode mode) {
  return static_cast<ControlMode>(
      ~static_cast<std::underlying_type_t<ControlMode>>(mode));
}

constexpr ControlMode& operator&=(ControlMode& left, ControlMode right) {
  return left = left & right;
}

constexpr ControlMode& operator|=(ControlMode& left, ControlMode right) {
  return left = left | right;
}

}  // namespace franka_hw
