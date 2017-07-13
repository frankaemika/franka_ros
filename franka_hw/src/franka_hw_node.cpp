#include <atomic>
#include <string>
#include <vector>

#include <controller_manager/controller_manager.h>
#include <ros/ros.h>
#include <ros/spinner.h>
#include <xmlrpcpp/XmlRpc.h>

#include <franka/robot.h>
#include <franka_hw/franka_hw.h>
#include <franka_hw/service_server.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "franka_hw");
  ros::NodeHandle node_handle("~");

  XmlRpc::XmlRpcValue params;
  node_handle.getParam("joint_names", params);
  std::vector<std::string> joint_names(params.size());
  for (int i = 0; i < params.size(); ++i) {
    joint_names[i] = static_cast<std::string>(params[i]);
  }
  std::string robot_ip;
  node_handle.getParam("robot_ip", robot_ip);
  std::string arm_id;
  node_handle.getParam("arm_id", arm_id);
  franka::Robot robot(robot_ip);

  // Set default collision behavior
  robot.setCollisionBehavior({{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
                             {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
                             {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
                             {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
                             {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
                             {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
                             {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
                             {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

  franka_hw::FrankaHW franka_control(joint_names, &robot, arm_id, node_handle);
  controller_manager::ControllerManager control_manager(&franka_control,
                                                        node_handle);

  std::atomic_bool has_error(false);
  franka_hw::ServiceServer service_server(
      robot, node_handle, {{"error_recovery", [&]() { has_error = false; }}});

  // Start background thread for message handling
  ros::AsyncSpinner spinner(1);
  spinner.start();

  while (ros::ok()) {
    ros::Time last_time = ros::Time::now();

    // Wait until controller has been activated or error has been recovered
    while (!franka_control.controllerActive() || has_error) {
      franka_control.readOnce();

      ros::Time now = ros::Time::now();
      control_manager.update(now, now - last_time);
      last_time = now;

      if (!ros::ok()) {
        return 0;
      }
    }

    // Reset controllers before starting a motion
    ros::Time now = ros::Time::now();
    control_manager.update(now, now - last_time, true);
    franka_control.reset();

    try {
      // Run control loop. Will exit if the controller is switched.
      franka_control.control(
          [&](const ros::Time& now, const ros::Duration& period) {
            control_manager.update(now, period);
            franka_control.enforceLimits(period);
            return ros::ok();
          });
    } catch (const franka::ControlException& e) {
      ROS_ERROR("%s", e.what());
      has_error = true;
    }
  }

  return 0;
}
