#include <franka_gazebo/controller_verifier.h>

namespace franka_gazebo {

ControllerVerifier::ControllerVerifier(
    const std::map<std::string, std::shared_ptr<franka_gazebo::Joint>>& joints,
    const std::string& arm_id)
    : arm_id_(arm_id) {
  for (const auto& joint : joints) {
    joint_names_.push_back(joint.first);
  }
}

bool ControllerVerifier::isValidController(
    const hardware_interface::ControllerInfo& controller) const {
  if (isClaimingGripperController(controller) or isClaimingArmController(controller)) {
    return true;
  }
  return std::none_of(
      controller.claimed_resources.begin(), controller.claimed_resources.end(),
      [](const auto& resource) {
        return ControllerVerifier::determineControlMethod(resource.hardware_interface);
      });
}

bool ControllerVerifier::isClaimingArmController(
    const hardware_interface::ControllerInfo& info) const {
  for (const auto& claimed_resource : info.claimed_resources) {
    if (hasControlMethodAndValidSize(claimed_resource)) {
      return areArmJoints(claimed_resource.resources);
    }
  }
  return false;
}

bool ControllerVerifier::isClaimingGripperController(
    const hardware_interface::ControllerInfo& info) const {
  for (const auto& claimed_resource : info.claimed_resources) {
    if (not areFingerJoints(claimed_resource.resources) or claimed_resource.resources.size() != 2) {
      continue;
    }
    auto control_method =
        ControllerVerifier::determineControlMethod(claimed_resource.hardware_interface);
    if (not control_method) {
      continue;
    }
    if (control_method.value() == EFFORT) {
      return true;
    }
  }
  return false;
}

bool ControllerVerifier::hasControlMethodAndValidSize(
    const hardware_interface::InterfaceResources& resource) {
  return ControllerVerifier::determineControlMethod(resource.hardware_interface)
             .is_initialized() and
         resource.resources.size() == 7;
}

bool ControllerVerifier::areArmJoints(const std::set<std::string>& resources) const {
  return std::all_of(resources.begin(), resources.end(), [this](const std::string& joint_name) {
    return std::find_if(joint_names_.begin(), joint_names_.end(),
                        [&joint_name, this](const std::string& joint) {
                          // make sure that the joint is not a finger joint
                          if (joint.find(arm_id_ + "_finger_joint") != std::string::npos) {
                            return false;
                          }
                          return joint == joint_name;
                        }) != joint_names_.end();
  });
}

bool ControllerVerifier::areFingerJoints(const std::set<std::string>& resources) const {
  return std::all_of(resources.begin(), resources.end(), [this](const std::string& joint_name) {
    return joint_name.find(arm_id_ + "_finger_joint") != std::string::npos;
  });
}

boost::optional<ControlMethod> ControllerVerifier::determineControlMethod(
    const std::string& hardware_interface) {
  if (hardware_interface.find("hardware_interface::PositionJointInterface") != std::string::npos) {
    return POSITION;
  }
  if (hardware_interface.find("hardware_interface::VelocityJointInterface") != std::string::npos) {
    return VELOCITY;
  }
  if (hardware_interface.find("hardware_interface::EffortJointInterface") != std::string::npos) {
    return EFFORT;
  }
  return boost::none;
}

}  // namespace franka_gazebo
