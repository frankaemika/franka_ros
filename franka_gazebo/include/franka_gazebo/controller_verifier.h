#pragma once
#include <franka_gazebo/joint.h>
#include <hardware_interface/controller_info.h>
#include <hardware_interface/interface_resources.h>
#include <hardware_interface/internal/hardware_resource_manager.h>
#include <boost/optional.hpp>

namespace franka_gazebo {

/// Can be used to check controllers in franka_gazebo. It also can distinguish
/// between gripper and arm controllers.
class ControllerVerifier {
 public:
  /// Creates a ControllerVerifier object to check controllers for franka_gazebo
  /// @param joints map of joint names and joints including gripper joints
  /// @param arm_id prefix of the joints
  ControllerVerifier(const std::map<std::string, std::shared_ptr<franka_gazebo::Joint>>& joints,
                     const std::string& arm_id);

  /// checks if a set of joint_names only contains the joints that are used for the arm
  bool areArmJoints(const std::set<std::string>& resources) const;

  /// checks if a controller can be used in the franka_hw gazebo plugin
  bool isValidController(const hardware_interface::ControllerInfo& controller) const;

  /// checks if a set of joint_names only contains the joints that are used for the gripper
  bool areFingerJoints(const std::set<std::string>& resources) const;

  /// checks if a controller wants to use the finger joints with the effort interface
  bool isClaimingGripperController(const hardware_interface::ControllerInfo& info) const;

  /// checks if a controller that uses the joints of the arm (not gripper joints) claims a position,
  /// velocity or effort interface.
  bool isClaimingArmController(const hardware_interface::ControllerInfo& info) const;

  /// returns the control method of a hardware interface
  static boost::optional<ControlMethod> determineControlMethod(
      const std::string& hardware_interface);

 private:
  std::vector<std::string> joint_names_;
  std::string arm_id_;

  static bool hasControlMethodAndValidSize(const hardware_interface::InterfaceResources& resource);
};
}  // namespace franka_gazebo
