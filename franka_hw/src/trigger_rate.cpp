#include <franka_hw/trigger_rate.h>

namespace franka_hw {

TriggerRate::TriggerRate(double rate)
    : rate_(rate), time_stamp_(ros::Time::now()) {}

bool TriggerRate::triggers() {
  if (elapsedCycleTime() > expectedCycleTime()) {
    time_stamp_ = ros::Time::now();
    return true;
  }
  return false;
}

}  // namespace franka_hw
