#include <franka_hw/trigger_rate.h>

namespace franka_hw {

TriggerRate::TriggerRate(double rate)
    : period_(1.0 / rate), time_stamp_(ros::Time::now()) {}

bool TriggerRate::triggers() {
  if ((ros::Time::now() - time_stamp_).toSec() > period_) {
    time_stamp_ = ros::Time::now();
    return true;
  }
  return false;
}

void TriggerRate::setRate(double rate) {
  period_ = 1.0 / rate;
  time_stamp_ = ros::Time::now();
}

}  // namespace franka_hw
