#include <ros/time.h>

namespace franka_hw {

class TriggerRate {
 public:
  explicit TriggerRate(double rate = 30.0);

  double elapsedCycleTime() const {
    return (ros::Time::now() - time_stamp_).toSec();
  }
  double expectedCycleTime() const { return 1.0 / rate_; }

  bool triggers();

 private:
  ros::Time time_stamp_;
  double rate_;
};

};  // namespace franka_hw
