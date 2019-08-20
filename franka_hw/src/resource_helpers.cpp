// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_hw/resource_helpers.h>

#include <ros/console.h>

namespace franka_hw {

bool findArmIdInResourceId(const std::string& resource_id, std::string* arm_id) {
  size_t position = resource_id.rfind("_joint");
  if (position != std::string::npos && position > 0) {
    *arm_id = resource_id.substr(0, position);
    return true;
  }
  position = resource_id.rfind("_robot");
  if (position != std::string::npos && position > 0) {
    *arm_id = resource_id.substr(0, position);
    return true;
  }
  return false;
}

ResourceWithClaimsMap getResourceMap(const std::list<hardware_interface::ControllerInfo>& info) {
  ResourceWithClaimsMap resource_map;
  for (auto& item : info) {
    const std::vector<hardware_interface::InterfaceResources>& c_res = item.claimed_resources;
    for (auto& resource_set : c_res) {
      const std::set<std::string>& iface_resources = resource_set.resources;
      for (auto& resource : iface_resources) {
        std::vector<std::string> claiming_controller(3);
        claiming_controller[0] = item.name;
        claiming_controller[1] = item.type;
        claiming_controller[2] = resource_set.hardware_interface;
        resource_map[resource].push_back(claiming_controller);
      }
    }
  }
  return resource_map;
}

bool getArmClaimedMap(ResourceWithClaimsMap& resource_map, ArmClaimedMap& arm_claim_map) {
  std::string current_arm_id;

  // check for conflicts between joint and cartesian level for each arm.
  // Valid claims are torque claims on joint level in combination with either
  // 7 non-torque claims on joint_level or one claim on cartesian level.
  for (auto map_it = resource_map.begin(); map_it != resource_map.end(); map_it++) {
    if (!findArmIdInResourceId(map_it->first, &current_arm_id)) {
      ROS_ERROR_STREAM("Resource conflict: Could not find arm_id in resource "
                       << map_it->first << ". Name joints as '<robot_arm_id>_joint<jointnumber>'");
      return false;
    }
    ResourceClaims new_claim;
    for (auto claimed_by : map_it->second) {
      if (claimed_by[2] == "hardware_interface::EffortJointInterface") {
        new_claim.joint_torque_claims++;
      } else if (claimed_by[2] == "hardware_interface::PositionJointInterface") {
        new_claim.joint_position_claims++;
      } else if (claimed_by[2] == "hardware_interface::VelocityJointInterface") {
        new_claim.joint_velocity_claims++;
      } else if (claimed_by[2] == "franka_hw::FrankaPoseCartesianInterface") {
        new_claim.cartesian_pose_claims++;
      } else if (claimed_by[2] == "franka_hw::FrankaVelocityCartesianInterface") {
        new_claim.cartesian_velocity_claims++;
      } else {
        return false;
      }
    }
    arm_claim_map[current_arm_id].joint_position_claims += new_claim.joint_position_claims;
    arm_claim_map[current_arm_id].joint_velocity_claims += new_claim.joint_velocity_claims;
    arm_claim_map[current_arm_id].joint_torque_claims += new_claim.joint_torque_claims;
    arm_claim_map[current_arm_id].cartesian_velocity_claims += new_claim.cartesian_velocity_claims;
    arm_claim_map[current_arm_id].cartesian_pose_claims += new_claim.cartesian_pose_claims;
  }
  return true;
}

ControlMode getControlMode(const std::string& arm_id, ArmClaimedMap& arm_claim_map) {
  ControlMode control_mode = ControlMode::None;
  if (arm_claim_map[arm_id].joint_position_claims > 0 &&
      arm_claim_map[arm_id].joint_velocity_claims == 0 &&
      arm_claim_map[arm_id].joint_torque_claims == 0 &&
      arm_claim_map[arm_id].cartesian_pose_claims == 0 &&
      arm_claim_map[arm_id].cartesian_velocity_claims == 0) {
    control_mode = ControlMode::JointPosition;
  } else if (arm_claim_map[arm_id].joint_position_claims == 0 &&
             arm_claim_map[arm_id].joint_velocity_claims > 0 &&
             arm_claim_map[arm_id].joint_torque_claims == 0 &&
             arm_claim_map[arm_id].cartesian_pose_claims == 0 &&
             arm_claim_map[arm_id].cartesian_velocity_claims == 0) {
    control_mode = ControlMode::JointVelocity;
  } else if (arm_claim_map[arm_id].joint_position_claims == 0 &&
             arm_claim_map[arm_id].joint_velocity_claims == 0 &&
             arm_claim_map[arm_id].joint_torque_claims > 0 &&
             arm_claim_map[arm_id].cartesian_pose_claims == 0 &&
             arm_claim_map[arm_id].cartesian_velocity_claims == 0) {
    control_mode = ControlMode::JointTorque;
  } else if (arm_claim_map[arm_id].joint_position_claims == 0 &&
             arm_claim_map[arm_id].joint_velocity_claims == 0 &&
             arm_claim_map[arm_id].joint_torque_claims == 0 &&
             arm_claim_map[arm_id].cartesian_pose_claims > 0 &&
             arm_claim_map[arm_id].cartesian_velocity_claims == 0) {
    control_mode = ControlMode::CartesianPose;
  } else if (arm_claim_map[arm_id].joint_position_claims == 0 &&
             arm_claim_map[arm_id].joint_velocity_claims == 0 &&
             arm_claim_map[arm_id].joint_torque_claims == 0 &&
             arm_claim_map[arm_id].cartesian_pose_claims == 0 &&
             arm_claim_map[arm_id].cartesian_velocity_claims > 0) {
    control_mode = ControlMode::CartesianVelocity;
  } else if (arm_claim_map[arm_id].joint_position_claims > 0 &&
             arm_claim_map[arm_id].joint_velocity_claims == 0 &&
             arm_claim_map[arm_id].joint_torque_claims > 0 &&
             arm_claim_map[arm_id].cartesian_pose_claims == 0 &&
             arm_claim_map[arm_id].cartesian_velocity_claims == 0) {
    control_mode = ControlMode::JointTorque | ControlMode::JointPosition;
  } else if (arm_claim_map[arm_id].joint_position_claims == 0 &&
             arm_claim_map[arm_id].joint_velocity_claims > 0 &&
             arm_claim_map[arm_id].joint_torque_claims > 0 &&
             arm_claim_map[arm_id].cartesian_pose_claims == 0 &&
             arm_claim_map[arm_id].cartesian_velocity_claims == 0) {
    control_mode = ControlMode::JointTorque | ControlMode::JointVelocity;
  } else if (arm_claim_map[arm_id].joint_position_claims == 0 &&
             arm_claim_map[arm_id].joint_velocity_claims == 0 &&
             arm_claim_map[arm_id].joint_torque_claims > 0 &&
             arm_claim_map[arm_id].cartesian_pose_claims > 0 &&
             arm_claim_map[arm_id].cartesian_velocity_claims == 0) {
    control_mode = ControlMode::JointTorque | ControlMode::CartesianPose;
  } else if (arm_claim_map[arm_id].joint_position_claims == 0 &&
             arm_claim_map[arm_id].joint_velocity_claims == 0 &&
             arm_claim_map[arm_id].joint_torque_claims > 0 &&
             arm_claim_map[arm_id].cartesian_pose_claims == 0 &&
             arm_claim_map[arm_id].cartesian_velocity_claims > 0) {
    control_mode = ControlMode::JointTorque | ControlMode::CartesianVelocity;
  }
  return control_mode;
}

bool hasConflictingMultiClaim(const ResourceWithClaimsMap& resource_map) {
  for (const auto& resource : resource_map) {
    if (resource.second.size() > 2) {
      ROS_ERROR_STREAM("Resource conflict: " << resource.first
                                             << " is claimed with more than two command interfaces "
                                                "which is not supported.");
      return true;
    }
    uint8_t torque_claims = 0;
    uint8_t other_claims = 0;
    if (resource.second.size() == 2) {
      for (const auto& claimed_by : resource.second) {
        if (claimed_by.at(2) == "hardware_interface::EffortJointInterface") {
          torque_claims++;
        } else {
          other_claims++;
        }
      }
      if (torque_claims != 1) {
        ROS_ERROR_STREAM("Resource conflict: "
                         << resource.first
                         << " is claimed with a combination of two interfaces that is not "
                            "supported.");
        return true;
      }
    }
  }
  return false;
}

bool hasConflictingJointAndCartesianClaim(const ArmClaimedMap& arm_claim_map,
                                          const std::string& arm_id) {
  // check for conflicts between joint and cartesian level for each arm.
  if (arm_claim_map.find(arm_id) != arm_claim_map.end()) {
    if ((arm_claim_map.at(arm_id).cartesian_velocity_claims +
                 arm_claim_map.at(arm_id).cartesian_pose_claims >
             0 &&
         arm_claim_map.at(arm_id).joint_position_claims +
                 arm_claim_map.at(arm_id).joint_velocity_claims >
             0)) {
      ROS_ERROR_STREAM(
          "Resource conflict: Invalid combination of claims on joint AND cartesian level on arm "
          << arm_id << " which is not supported.");
      return true;
    }
  }
  return false;
}

bool partiallyClaimsArmJoints(const ArmClaimedMap& arm_claim_map, const std::string& arm_id) {
  // Valid claims are torque claims on joint level in combination with either
  // 7 non-torque claims on joint_level or one claim on cartesian level.
  if (arm_claim_map.find(arm_id) != arm_claim_map.end()) {
    if ((arm_claim_map.at(arm_id).joint_position_claims > 0 &&
         arm_claim_map.at(arm_id).joint_position_claims != 7) ||
        (arm_claim_map.at(arm_id).joint_velocity_claims > 0 &&
         arm_claim_map.at(arm_id).joint_velocity_claims != 7) ||
        (arm_claim_map.at(arm_id).joint_torque_claims > 0 &&
         arm_claim_map.at(arm_id).joint_torque_claims != 7)) {
      ROS_ERROR_STREAM("Resource conflict: Partially claiming joints of arm "
                       << arm_id
                       << " is not supported. Make sure to claim all 7 joints of the robot.");
      return true;
    }
  }
  return false;
}

bool hasTrajectoryClaim(const ArmClaimedMap& arm_claim_map, const std::string& arm_id) {
  if (arm_claim_map.find(arm_id) != arm_claim_map.end()) {
    if (arm_claim_map.at(arm_id).joint_position_claims +
            arm_claim_map.at(arm_id).joint_velocity_claims +
            arm_claim_map.at(arm_id).cartesian_velocity_claims +
            arm_claim_map.at(arm_id).cartesian_pose_claims >
        0) {
      return true;
    }
  }
  return false;
}

}  // namespace franka_hw
