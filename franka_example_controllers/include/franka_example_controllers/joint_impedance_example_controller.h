#pragma once

#include <memory>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/node_handle.h>
#include <ros/time.h>

#include <franka_example_controllers/JointTorqueComparison.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/trigger_rate.h>

namespace franka_example_controllers {

class JointImpedanceExampleController
    : public controller_interface::MultiInterfaceController<
          franka_hw::FrankaModelInterface,
          hardware_interface::EffortJointInterface,
          franka_hw::FrankaPoseCartesianInterface> {
 public:
  JointImpedanceExampleController();
  bool init(hardware_interface::RobotHW* robot_hw,
            ros::NodeHandle& node_handle);
  void update(const ros::Time&, const ros::Duration& period);
  void stopping(const ros::Time&);

 private:
  franka_hw::FrankaModelInterface* model_interface_;
  franka_hw::FrankaPoseCartesianInterface* cartesian_pose_interface_;
  hardware_interface::EffortJointInterface* effort_joint_interface_;
  std::unique_ptr<franka_hw::FrankaCartesianPoseHandle> cartesian_pose_handle_;
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  std::vector<hardware_interface::JointHandle> joint_handles_;
  std::array<double, 7> last_tau_d_ = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  franka_hw::TriggerRate rate_trigger_;
  std::string arm_id_;
  double radius_{0.1};
  double acceleration_time_{2.0};
  double vel_max_{0.05};
  std::vector<std::string> joint_names_;
  std::array<double, 7> k_gains_;
  std::array<double, 7> d_gains_;
  double angle_{0.0};
  double vel_current_{0.0};
  std::array<double, 16> initial_pose_;
  realtime_tools::RealtimePublisher<JointTorqueComparison> torques_publisher_;
};

}  // namespace franka_example_controllers
