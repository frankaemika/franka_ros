#pragma once

#include <franka_hw/franka_controller_switching_types.h>

namespace franka_hw {

bool findArmIDinResourceID(const std::string& resource_id, std::string* arm_id);

ResourceWithClaimsMap getResourceMap(
    const std::list<hardware_interface::ControllerInfo>& info);

bool getArmClaimedMap(ResourceWithClaimsMap& resource_map,
                      ArmClaimedMap& arm_claim_map);

}  // namespace franka_hw
