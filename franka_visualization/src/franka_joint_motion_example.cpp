
#include <franka/robot.h>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <cmath>

int main(int argc, char** argv) {
  ros::init(argc, argv, "franka_joint_state_publisher");
  ros::NodeHandle private_nodehandle("~");
  ros::NodeHandle public_nodehandle;
  ros::Rate rate(1000);
  ros::Publisher joint_pub =
      public_nodehandle.advertise<sensor_msgs::JointState>("joint_states", 1);
  std::vector<std::string> joint_names;

  XmlRpc::XmlRpcValue params;
  private_nodehandle.getParam("joint_names", params);
  joint_names.resize(params.size());
  for (int i = 0; i < params.size(); ++i) {
    joint_names[i] = static_cast<std::string>(params[i]);
    ROS_INFO("parsed jointname[%d]= %s", i, joint_names[i].c_str());
  }
  std::string robot_ip;
  private_nodehandle.getParam("robot_ip", robot_ip);
  ROS_INFO("parsed franka robot IP: %s", robot_ip.c_str());

  sensor_msgs::JointState states;
  states.effort.resize(joint_names.size());
  states.name.resize(joint_names.size());
  states.position.resize(joint_names.size());
  states.velocity.resize(joint_names.size());

  try {
    ROS_INFO("connecting to robot... ");
    franka::Robot robot(robot_ip);
    ROS_INFO("Starting joint pose motion generator");
    franka::JointPoseMotionGenerator motion_generator =
        robot.startJointPoseMotionGenerator();
    ROS_INFO("succeeded to start motion generator! ");

    double time(0.0);
    std::array<double, 7> initial_pose;
    std::copy(robot.robotState().q.cbegin(), robot.robotState().q.cend(),
              initial_pose.begin());

    uint64_t sequence_number = 1;
    sleep(1);
    ROS_INFO("Starting control loop");

    while (ros::ok() && robot.update()) {
      if (std::fabs(std::fmod(time, 1.0 / 30.0)) < 0.001) {
        const franka::RobotState& robot_state = robot.robotState();
        states.header.stamp = ros::Time::now();
        states.header.seq = sequence_number;
        for (int i = 0; i < joint_names.size(); ++i) {
          states.name[i] = joint_names[i];
          states.position[i] = robot_state.q[i];
          states.velocity[i] = robot_state.dq[i];
          states.effort[i] = robot_state.tau_J[i];
        }
        joint_pub.publish(states);
        ros::spinOnce();
        rate.sleep();
        sequence_number++;
        ROS_INFO("published joint states");
      }

      double delta_angle = M_PI / 8 * (1 - std::cos(M_PI / 5.0 * time));
      try {
        motion_generator.setDesiredPose(
            {{initial_pose[0], initial_pose[1], initial_pose[2],
              initial_pose[3] + delta_angle, initial_pose[4] + delta_angle,
              initial_pose[5], initial_pose[6] + delta_angle}});
      } catch (franka::MotionGeneratorException const& e) {
        std::cout << e.what() << std::endl;
      }

      ROS_INFO_STREAM("Set desired pose:"
                      << initial_pose[0] << " " << initial_pose[1] << " "
                      << initial_pose[2] << " " << initial_pose[3] + delta_angle
                      << " " << initial_pose[4] + delta_angle << " "
                      << initial_pose[5] << " "
                      << initial_pose[6] + delta_angle);
      time += 0.001;
      if (time > 10.0) {
        ROS_INFO("Finished motion, shutting down example");
        break;
      }
    }
  } catch (franka::NetworkException const& e) {
    ROS_ERROR_STREAM("" << e.what());
    return -1;
  }
  return 0;
}
