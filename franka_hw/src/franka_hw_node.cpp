#include <atomic>
#include <string>
#include <utility>
#include <vector>

#include <controller_manager/controller_manager.h>
#include <ros/ros.h>
#include <ros/spinner.h>
#include <xmlrpcpp/XmlRpc.h>

#include <franka/robot.h>
#include <franka_hw/franka_hw.h>
#include <franka_hw/services.h>

class ServiceContainer {
 public:
  template <typename T, typename... TArgs>
  ServiceContainer& advertiseService(TArgs&&... args) {
    ros::ServiceServer server =
        franka_hw::services::advertiseService<T>(std::forward<TArgs>(args)...);
    services_.push_back(server);
    return *this;
  }

 private:
  std::vector<ros::ServiceServer> services_;
};

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

  std::atomic_bool has_error(false);

  using std::placeholders::_1;
  using std::placeholders::_2;
  ServiceContainer services;
  services
      .advertiseService<franka_hw::SetJointImpedance>(
          node_handle, "set_joint_impedance",
          std::bind(franka_hw::services::setJointImpedance, std::ref(robot), _1,
                    _2))
      .advertiseService<franka_hw::SetCartesianImpedance>(
          node_handle, "set_cartesian_impedance",
          std::bind(franka_hw::services::setCartesianImpedance, std::ref(robot),
                    _1, _2))
      .advertiseService<franka_hw::SetEEFrame>(
          node_handle, "set_EE_frame",
          std::bind(franka_hw::services::setEEFrame, std::ref(robot), _1, _2))
      .advertiseService<franka_hw::SetKFrame>(
          node_handle, "set_K_frame",
          std::bind(franka_hw::services::setKFrame, std::ref(robot), _1, _2))
      .advertiseService<franka_hw::SetForceTorqueCollisionBehavior>(
          node_handle, "set_force_torque_collision_behavior",
          std::bind(franka_hw::services::setForceTorqueCollisionBehavior,
                    std::ref(robot), _1, _2))
      .advertiseService<franka_hw::SetFullCollisionBehavior>(
          node_handle, "set_full_collision_behavior",
          std::bind(franka_hw::services::setFullCollisionBehavior,
                    std::ref(robot), _1, _2))
      .advertiseService<franka_hw::SetLoad>(
          node_handle, "set_load",
          std::bind(franka_hw::services::setLoad, std::ref(robot), _1, _2))
      .advertiseService<franka_hw::SetTimeScalingFactor>(
          node_handle, "set_time_scaling_factor",
          std::bind(franka_hw::services::setTimeScalingFactor, std::ref(robot),
                    _1, _2))
      .advertiseService<franka_hw::ErrorRecovery>(
          node_handle, "error_recovery",
          [&](const franka_hw::ErrorRecovery::Request& req,
              franka_hw::ErrorRecovery::Response& res) {
            franka_hw::services::errorRecovery(robot, req, res);
            has_error = false;
          });

  franka_hw::FrankaHW franka_control(joint_names, arm_id, node_handle);

  // Initialize robot state before loading any controller
  franka_control.update(robot.readOnce());

  controller_manager::ControllerManager control_manager(&franka_control,
                                                        node_handle);

  // Start background thread for message handling
  ros::AsyncSpinner spinner(1);
  spinner.start();

  while (ros::ok()) {
    ros::Time last_time = ros::Time::now();

    // Wait until controller has been activated or error has been recovered
    while (!franka_control.controllerActive() || has_error) {
      franka_control.update(robot.readOnce());

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
    franka_control.update(robot.readOnce(), true);

    try {
      // Run control loop. Will exit if the controller is switched.
      franka_control.control(
          robot, [&](const ros::Time& now, const ros::Duration& period) {
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
