#pragma once

#include <angles/angles.h>
#include <franka/robot_state.h>
#include <franka_hw/franka_state_interface.h>
#include <gazebo_ros_control/robot_hw_sim.h>
#include <hardware_interface/internal/hardware_resource_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <urdf/model.h>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <memory>

namespace franka_gazebo {

struct Joint {
  Joint() = default;
  Joint(Joint&&) = default;
  Joint(const Joint&) = delete;

  std::string name;
  gazebo::physics::JointPtr handle;
  int type;
  double command;
  double position;
  double velocity;
  double effort;

  void update() {
    if (not this->handle)
      return;
    this->velocity = this->handle->GetVelocity(0);
    this->effort = this->handle->GetForce(0);
    double position = this->handle->Position(0);
    switch (this->type) {
      case urdf::Joint::PRISMATIC:
        this->position = position;
        break;
      case urdf::Joint::REVOLUTE:
      case urdf::Joint::CONTINUOUS:
        this->position += angles::shortest_angular_distance(this->position, position);
        break;
    }
  }
};

class FrankaHWSim : public gazebo_ros_control::RobotHWSim {
 public:
  bool initSim(const std::string& robot_namespace,
               ros::NodeHandle model_nh,
               gazebo::physics::ModelPtr parent,
               const urdf::Model* const urdf,
               std::vector<transmission_interface::TransmissionInfo> transmissions) override;

  void readSim(ros::Time time, ros::Duration period) override;
  void writeSim(ros::Time time, ros::Duration period) override;

  void eStopActive(const bool active) override;

 private:
  std::vector<franka_gazebo::Joint> joints_;

  hardware_interface::JointStateInterface jsi_;
  hardware_interface::EffortJointInterface eji_;
  franka_hw::FrankaStateInterface fsi_;

  franka::RobotState robot_state_;

  double todo = 0;  // joint position + velocity + effort
};

}  // namespace franka_gazebo
