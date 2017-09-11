#pragma once

#include <ostream>

namespace franka_hw {

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
  return static_cast<ControlMode>(static_cast<std::underlying_type_t<ControlMode>>(left) &
                                  static_cast<std::underlying_type_t<ControlMode>>(right));
}

constexpr ControlMode operator|(ControlMode left, ControlMode right) {
  return static_cast<ControlMode>(static_cast<std::underlying_type_t<ControlMode>>(left) |
                                  static_cast<std::underlying_type_t<ControlMode>>(right));
}

constexpr ControlMode operator^(ControlMode left, ControlMode right) {
  return static_cast<ControlMode>(static_cast<std::underlying_type_t<ControlMode>>(left) ^
                                  static_cast<std::underlying_type_t<ControlMode>>(right));
}

constexpr ControlMode operator~(ControlMode mode) {
  return static_cast<ControlMode>(~static_cast<std::underlying_type_t<ControlMode>>(mode));
}

constexpr ControlMode& operator&=(ControlMode& left, ControlMode right) {
  return left = left & right;
}

constexpr ControlMode& operator|=(ControlMode& left, ControlMode right) {
  return left = left | right;
}

}  // namespace franka_hw
