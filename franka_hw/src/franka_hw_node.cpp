#include <ros/ros.h>

#include <franka_hw/franka_hw.h>

#include <franka/robot.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "franka_hw_node");

 franka_hw::FrankaHW franka_ros;

  ros::NodeHandle nh("~");
  franka_ros.init(nh);

  while (ros::ok())
  {
    if(franka_ros.update() == false)
    {
        ROS_ERROR("failed to update franka_hw. Shutting down hardware node!");
        return -1;
    }
  }

  return 0;
}
