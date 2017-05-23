#pragma once

#include <list>
#include <map>
#include <set>
#include <string>
#include <vector>

#include <hardware_interface/controller_info.h>
#include <hardware_interface/interface_resources.h>

namespace franka_hw {

typedef std::map<std::string, std::vector<std::vector<std::string> > >
    ResourceWithClaimsMap;

typedef std::list<hardware_interface::ControllerInfo>::const_iterator
    ControlInfoIterator;

typedef std::vector<hardware_interface::InterfaceResources>::const_iterator
    ClaimedResourceIterator;

typedef std::set<std::string>::const_iterator ResourceIterator;

typedef std::map<std::string, std::vector<std::vector<std::string> > >::iterator
    ResourceMapIterator;

typedef struct {
  uint8_t joint_position_claims = 0;
  uint8_t joint_velocity_claims = 0;
  uint8_t joint_torque_claims = 0;
  uint8_t cartesian_velocity_claims = 0;
  uint8_t cartesian_pose_claims = 0;
} ResourceClaims;

typedef std::map<std::string, ResourceClaims> ArmClaimedMap;

typedef enum {
  NonValidRequest = 0,
  JointTorque = 1,
  JointPosition = 2,
  JointVelocity = 3,
  CartesianVelocity = 4,
  CartesianPose = 5,
  TorqueAndJointPosition = 6,
  TorqueAndJointVelocity = 7,
  TorqueAndCartesianPose = 8,
  TorqueAndCartesianVelocity = 9
} RequestedControl;

}  // namespace franka_hw
