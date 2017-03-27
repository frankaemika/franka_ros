
#include <ros/ros.h>

namespace franka_hw {

class TriggerRate {
 public:
  explicit TriggerRate(double rate = 30.0)
      : rate_(rate), time_stamp_(ros::Time::now()) {}
  ~TriggerRate() {}
  bool triggers() {
    if (elapsedCycleTime() > expectedCycleTime()) {
      time_stamp_ = ros::Time::now();
      return true;
    }
    return false;
  }
  double elapsedCycleTime() { return (ros::Time::now() - time_stamp_).toSec(); }
  double expectedCycleTime() { return 1.0 / rate_; }

 private:
  ros::Time time_stamp_;
  double rate_;
};

};  // namespace franka_hw
