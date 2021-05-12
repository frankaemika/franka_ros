#pragma once

#include <franka/model.h>
#include <franka/robot_state.h>
#include <franka_gazebo/joint.h>
#include <franka_gazebo/model_kdl.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <gazebo_ros_control/robot_hw_sim.h>
#include <hardware_interface/internal/hardware_resource_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <urdf/model.h>
#include <cmath>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <map>
#include <memory>

namespace franka_gazebo {

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

  void updateRobotState(ros::Time time);

  bool readParameters(ros::NodeHandle nh);
  std::array<double, 9> readArray9(std::string values, std::string name = "");
  std::array<double, 3> readArray3(std::string values, std::string name = "");

 private:
  gazebo::physics::ModelPtr robot_;
  std::vector<std::string> names_;
  std::map<std::string, std::shared_ptr<franka_gazebo::Joint>> joints_;

  hardware_interface::JointStateInterface jsi_;
  hardware_interface::EffortJointInterface eji_;
  franka_hw::FrankaStateInterface fsi_;
  franka_hw::FrankaModelInterface fmi_;

  franka::RobotState robot_state_;
  franka::ModelPtr model_;

  template <int N>
  std::array<double, N> readArray(std::string param, std::string name = "") {
    std::array<double, N> x;

    std::istringstream iss(param);
    std::vector<std::string> values{std::istream_iterator<std::string>{iss},
                                    std::istream_iterator<std::string>{}};
    if (values.size() != N) {
      throw std::invalid_argument("Expected parameter '" + name + "' to have exactely " +
                                  std::to_string(N) + " numbers separated by spaces, but found " +
                                  std::to_string(values.size()));
    }
    std::transform(values.begin(), values.end(), x.begin(),
                   [](std::string v) -> double { return std::stod(v); });
    return x;
  }
};

}  // namespace franka_gazebo
