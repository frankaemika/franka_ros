
#include <ros/ros.h>

#include <franka_hw/franka_hw_helper_functions.h>

namespace franka_hw {

bool findArmIDinResourceID(const std::string& resource_id,
                           std::string& arm_id) {
  size_t position = resource_id.rfind("_joint");
  if (position != std::string::npos && position > 0) {
    arm_id = resource_id.substr(0, position);
    return true;
  }
  position = resource_id.rfind("_cartesian");
  if (position != std::string::npos && position > 0) {
    arm_id = resource_id.substr(0, position);
    return true;
  }
  return false;
}

ResourceWithClaimsMap getResourceMap(
    const std::list<hardware_interface::ControllerInfo>& info) {
  ResourceWithClaimsMap resource_map;
  for (ControlInfoIterator info_it = info.begin(); info_it != info.end();
       ++info_it) {
    const std::vector<hardware_interface::InterfaceResources>& c_res =
        info_it->claimed_resources;
    for (ClaimedResourceIterator c_res_it = c_res.begin();
         c_res_it != c_res.end(); ++c_res_it) {
      const std::set<std::string>& iface_resources = c_res_it->resources;
      for (ResourceIterator resource_it = iface_resources.begin();
           resource_it != iface_resources.end(); ++resource_it) {
        std::vector<std::string> claiming_controller(3);
        claiming_controller[0] = info_it->name;
        claiming_controller[1] = info_it->type;
        claiming_controller[2] = c_res_it->hardware_interface;
        resource_map[*resource_it].push_back(claiming_controller);
      }
    }
  }
  return resource_map;
}

bool getArmClaimedMap(ResourceWithClaimsMap& resource_map,
                      ArmClaimedMap& arm_claim_map) {
  std::string current_arm_id;

  // check for conflicts between joint and cartesian level for each arm.
  // Valid claims are torque claims on joint level in combination with either
  // 7 non-torque claims on joint_level or one claim on cartesian level.
  for (ResourceMapIterator map_it = resource_map.begin();
       map_it != resource_map.end(); map_it++) {
    if (!findArmIDinResourceID(map_it->first, current_arm_id)) {
      ROS_ERROR_STREAM("Could not find arm_id in resource "
                       << map_it->first
                       << ".Conflict! \n Name joints as "
                          "'<robot_arm_id>_joint<jointnumber>'");
      return true;
    }
    ResourceClaims new_claim;
    for (std::vector<std::vector<std::string> >::iterator claimed_by_it =
             map_it->second.begin();
         claimed_by_it != map_it->second.end(); ++claimed_by_it) {
      if ((*claimed_by_it)[2] == "hardware_interface::EffortJointInterface") {
        new_claim.joint_torque_claims++;
      } else if ((*claimed_by_it)[2] ==
                 "hardware_interface::PositionJointInterface") {
        new_claim.joint_position_claims++;
      } else if ((*claimed_by_it)[2] ==
                 "hardware_interface::VelocityJointInterface") {
        new_claim.joint_velocity_claims++;
      } else if ((*claimed_by_it)[2] ==
                 "franka_hw::FrankaPoseCartesianInterface") {
        new_claim.cartesian_pose_claims++;
      } else if ((*claimed_by_it)[2] ==
                 "franka_hw::FrankaVelocityCartesianInterface") {
        new_claim.cartesian_velocity_claims++;
      } else {
        return false;
      }
    }
    arm_claim_map[current_arm_id].joint_position_claims +=
        new_claim.joint_position_claims;
    arm_claim_map[current_arm_id].joint_velocity_claims +=
        new_claim.joint_velocity_claims;
    arm_claim_map[current_arm_id].joint_torque_claims +=
        new_claim.joint_torque_claims;
    arm_claim_map[current_arm_id].cartesian_velocity_claims +=
        new_claim.cartesian_velocity_claims;
    arm_claim_map[current_arm_id].cartesian_pose_claims +=
        new_claim.cartesian_pose_claims;
  }
  return true;
}

}  // namespace franka_hw
