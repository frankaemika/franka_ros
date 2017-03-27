
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf2_msgs/TFMessage.h>

namespace franka_hw {

class RealTimeTfPublisher {
 public:
  RealTimeTfPublisher() {}
  explicit RealTimeTfPublisher(const ros::NodeHandle& nh)
      : publisher_tf_(nh, "/tf", 1) {}
  void init(const ros::NodeHandle& nh) { publisher_tf_.init(nh, "/tf", 1); }
  bool tryLock() { return publisher_tf_.trylock(); }
  void unlockAndPublish() { publisher_tf_.unlockAndPublish(); }
  void lock() { publisher_tf_.lock(); }
  void unlock() { publisher_tf_.unlock(); }
  void setTransform(const geometry_msgs::TransformStamped& transform);
  void setTransform(const std::array<double, 16>& transform,
                    const std::string& child_frame_id,
                    const std::string& frame_id);
  void setTransform(const tf::StampedTransform& transform);

 private:
  realtime_tools::RealtimePublisher<tf2_msgs::TFMessage> publisher_tf_;
};

}  // namespace franka_hw
