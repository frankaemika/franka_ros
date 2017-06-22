#pragma once

#include <list>
#include <map>
#include <string>
#include <vector>

#include <hardware_interface/controller_info.h>

#include <franka_hw/franka_controller_switching_types.h>

namespace franka_hw {

bool findArmIdInResourceId(const std::string& resource_id, std::string* arm_id);

ResourceWithClaimsMap getResourceMap(
    const std::list<hardware_interface::ControllerInfo>& info);

bool getArmClaimedMap(ResourceWithClaimsMap& resource_map,
                      ArmClaimedMap& arm_claim_map);

ControlMode getControlMode(const std::string& arm_id,
                           ArmClaimedMap& arm_claim_map);

}  // namespace franka_hw
