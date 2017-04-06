#include <cinttypes>

#include <franka/robot.h>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "franka_joint_state_publisher");
  ros::NodeHandle private_nodehandle("~");
  ros::NodeHandle public_nodehandle;
  ros::Rate rate(30.0);
  ros::Publisher joint_pub =
      public_nodehandle.advertise<sensor_msgs::JointState>("joint_states", 1);
  std::vector<std::string> joint_names;
  XmlRpc::XmlRpcValue params;
  private_nodehandle.getParam("joint_names", params);
  const size_t kNumberOfJoints = params.size();
  joint_names.resize(kNumberOfJoints);

  for (size_t i = 0; i < kNumberOfJoints; i++) {
    joint_names[i] = static_cast<std::string>(params[i]);
    ROS_INFO_STREAM("parsed jointname[" << i << "]= " << joint_names[i]);
  }

  std::string robot_ip;
  private_nodehandle.getParam("robot_ip", robot_ip);
  ROS_INFO("parsed franka robot IP: %s", robot_ip.c_str());

  sensor_msgs::JointState states;
  states.effort.resize(kNumberOfJoints);
  states.name.resize(kNumberOfJoints);
  states.position.resize(kNumberOfJoints);
  states.velocity.resize(kNumberOfJoints);

  for (size_t i = 0; i < joint_names.size(); i++) {
    states.name[i] = joint_names[i];
  }

  try {
    ROS_INFO("connecting to robot... ");
    franka::Robot robot(robot_ip);

    robot.read([sequence_number = 1ul,
                kNumberOfJoints,
                &states,
                &rate,
                &joint_pub](const franka::RobotState& robot_state) mutable {
      states.header.stamp = ros::Time::now();
      states.header.seq = sequence_number++;
      for (size_t i = 0; i < robot_state.q.size(); i++) {
        states.position[i] = robot_state.q[i];
        states.velocity[i] = robot_state.dq[i];
        states.effort[i] = robot_state.tau_J[i];
      }
      joint_pub.publish(states);
      ros::spinOnce();
      rate.sleep();
      return ros::ok();
    });

  } catch (const franka::Exception& e) {
    ROS_ERROR_STREAM("" << e.what());
    return -1;
  }

  return 0;
}
