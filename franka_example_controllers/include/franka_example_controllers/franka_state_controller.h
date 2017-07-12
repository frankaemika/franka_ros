#pragma once

#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/FrankaState.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_publisher.h>
#include <sensor_msgs/JointState.h>
#include <tf2_msgs/TFMessage.h>

#include <franka_hw/franka_state_interface.h>
#include <franka_hw/trigger_rate.h>

namespace franka_example_controllers {

class FrankaStateController
    : public controller_interface::MultiInterfaceController<
          franka_hw::FrankaStateInterface> {
 public:
  FrankaStateController();
  bool init(hardware_interface::RobotHW* robot_hw,
            ros::NodeHandle& node_handle);
  void update(const ros::Time& time, const ros::Duration&);

 private:
  /**
  * Publishes all relevant data received from the Franka arm
  */
  void publishFrankaStates();

  /**
  * Publishes the joint states of the Franka arm
  */
  void publishJointStates();

  /**
  * Publishes the transforms for EE and K frame which define the end-effector
  * (EE) and the Cartesian impedance reference frame (K)
  */
  void publishTransforms();

  /**
  * Publishes the estimated external wrench felt by the Franka
  */
  void publishExternalWrench();

  std::string arm_id_;

  franka_hw::FrankaStateInterface* franka_state_interface_;
  franka_hw::FrankaStateHandle* franka_state_handle_;

  realtime_tools::RealtimePublisher<tf2_msgs::TFMessage> publisher_transforms_;
  realtime_tools::RealtimePublisher<franka_hw::FrankaState>
      publisher_franka_states_;
  realtime_tools::RealtimePublisher<sensor_msgs::JointState>
      publisher_joint_states_;
  realtime_tools::RealtimePublisher<geometry_msgs::WrenchStamped>
      publisher_external_wrench_;
  franka_hw::TriggerRate trigger_publish_;
  franka::RobotState robot_state_;
  uint64_t sequence_number_ = 0;
  std::vector<std::string> joint_names_;
};

}  // namespace franka_example_controllers
