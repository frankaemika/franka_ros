#pragma once

#include <cstdint>
#include <memory>
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

namespace franka_hw {

class FrankaStateController
    : public controller_interface::MultiInterfaceController<
          franka_hw::FrankaStateInterface> {
 public:
  FrankaStateController();

  /**
  * Initializes the controller with interfaces and publishers
  *
  * @param[in] hardware Pointer to the robot hardware
  * @param[in] root_node_handle Nodehandle on root level passed from HW node
  * @param[in] controller_node_handle Nodehandle in the controller namespace
  * passed from HW node
  */
  bool init(hardware_interface::RobotHW* robot_hardware,
            ros::NodeHandle& root_node_handle,
            ros::NodeHandle& controller_node_handle) override;

  /**
  * Reads a new franka robot state and publishes it
  *
  * @param[in] time Current ros time
  */
  void update(const ros::Time& time, const ros::Duration&) override;

 private:
  /**
  * Publishes all relevant data received from the Franka arm
  *
  * @param[in] time Current ros time
  */
  void publishFrankaStates(const ros::Time& time);

  /**
  * Publishes the joint states of the Franka arm
  *
  * @param[in] time Current ros time
  */
  void publishJointStates(const ros::Time& time);

  /**
  * Publishes the transforms for EE and K frame which define the end-effector
  * (EE) and the Cartesian impedance reference frame (K)
  *
  * @param[in] time Current ros time
  */
  void publishTransforms(const ros::Time& time);

  /**
  * Publishes the estimated external wrench felt by the Franka
  *
  * @param[in] time Current ros time
  */
  void publishExternalWrench(const ros::Time& time);

  std::string arm_id_;

  franka_hw::FrankaStateInterface* franka_state_interface_;
  std::unique_ptr<franka_hw::FrankaStateHandle> franka_state_handle_;

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

}  // namespace franka_hw
