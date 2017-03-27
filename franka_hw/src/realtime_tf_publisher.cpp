#include <franka_hw/realtime_tf_publisher.h>

void franka_hw::RealTimeTfPublisher::setTransform(
    const geometry_msgs::TransformStamped& transform) {
  publisher_tf_.msg_.transforms.resize(1);
  publisher_tf_.msg_.transforms[0] = transform;
}

void franka_hw::RealTimeTfPublisher::setTransform(
    const tf::StampedTransform& transform) {
  geometry_msgs::TransformStamped transform_message;
  transformStampedTFToMsg(transform, transform_message);
  setTransform(transform_message);
}

void franka_hw::RealTimeTfPublisher::setTransform(
    const std::array<double, 16>& transform,
    const std::string& child_frame_id,
    const std::string frame_id) {
  geometry_msgs::TransformStamped transform_message;
  transform_message.header.stamp = ros::Time::now();
  transform_message.child_frame_id = child_frame_id;
  transform_message.header.frame_id = frame_id;
  transform_message.transform.translation.x = transform[12];
  transform_message.transform.translation.y = transform[13];
  transform_message.transform.translation.z = transform[14];
  tf::Matrix3x3 rotation(transform[0], transform[4], transform[8], transform[1],
                         transform[5], transform[9], transform[2], transform[6],
                         transform[10]);
  tf::Quaternion quaternion;
  rotation.getRotation(quaternion);
  geometry_msgs::Quaternion quaternion_message;
  tf::quaternionTFToMsg(quaternion, quaternion_message);
  transform_message.transform.rotation = quaternion_message;
  setTransform(transform_message);
}
