// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_hw/control_mode.h>

#include <algorithm>
#include <iterator>
#include <string>
#include <vector>

namespace franka_hw {

std::ostream& operator<<(std::ostream& ostream, ControlMode mode) {
  if (mode == ControlMode::None) {
    ostream << "<none>";
  } else {
    std::vector<std::string> names;
    if ((mode & ControlMode::JointTorque) != ControlMode::None) {
      names.emplace_back("JointTorque");
    }
    if ((mode & ControlMode::JointPosition) != ControlMode::None) {
      names.emplace_back("JointPosition");
    }
    if ((mode & ControlMode::JointVelocity) != ControlMode::None) {
      names.emplace_back("JointVelocity");
    }
    if ((mode & ControlMode::CartesianVelocity) != ControlMode::None) {
      names.emplace_back("CartesianVelocity");
    }
    if ((mode & ControlMode::CartesianPose) != ControlMode::None) {
      names.emplace_back("CartesianPose");
    }
    std::copy(names.cbegin(), names.cend() - 1, std::ostream_iterator<std::string>(ostream, ", "));
    ostream << names.back();
  }
  return ostream;
}

}  // namespace franka_hw
