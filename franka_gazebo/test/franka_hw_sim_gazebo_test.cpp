#include <geometry_msgs/WrenchStamped.h>
#include <gtest/gtest.h>
#include <ros/ros.h>

TEST(
#if ROS_VERSION_MINIMUM(1, 15, 13)
    TestSuite, /* noetic & newer */
#else
    DISABLED_TestSuite, /* melodic */
#endif
    franka_hw_sim_compensates_gravity_on_F_ext) {
  ros::NodeHandle n;

  for (int i = 0; i < 50; i++) {
    auto msg = ros::topic::waitForMessage<geometry_msgs::WrenchStamped>(
        "/franka_state_controller/F_ext", n);

    auto now = msg->header.stamp;

    EXPECT_LE(std::abs(msg->wrench.force.x), 0.5) << "During time: " << now;
    EXPECT_LE(std::abs(msg->wrench.force.y), 0.5) << "During time: " << now;
    EXPECT_LE(std::abs(msg->wrench.force.z), 0.5) << "During time: " << now;
    EXPECT_LE(std::abs(msg->wrench.torque.x), 0.25) << "During time: " << now;
    EXPECT_LE(std::abs(msg->wrench.torque.y), 0.25) << "During time: " << now;
    EXPECT_LE(std::abs(msg->wrench.torque.z), 0.25) << "During time: " << now;
  }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "franka_hw_sim_test");
  return RUN_ALL_TESTS();
}
