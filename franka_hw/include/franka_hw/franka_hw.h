#pragma once

#include <string>
#include <array>
#include <vector>
#include <pluginlib/class_list_macros.h>

#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_publisher.h>
#include <sensor_msgs/JointState.h>

#include <franka/robot.h>

#include <franka_hw/FrankaState.h>
#include <franka_hw/franka_cartesian_state_interface.h>
#include <franka_hw/franka_joint_state_interface.h>


namespace franka_hw {

class FrankaHW : public hardware_interface::RobotHW {
public:
    FrankaHW();
    FrankaHW(const std::vector<std::string> joint_names, const std::string ip);
    ~FrankaHW();
    bool update();
    void publishFrankaStates();
    void publishJointStates();
    void updateStates(const franka::RobotState& robot_state);

private:
    hardware_interface::JointStateInterface jnt_state_interface_;  // interfaces
    hardware_interface::FrankaJointStateInterface franka_jnt_state_interface_;
    hardware_interface::FrankaCartesianStateInterface
    franka_cart_state_interface_;

    franka::Robot robot_;  // libfranka robot

    realtime_tools::RealtimePublisher<franka_hw::FrankaState>* pub_franka_states_;
    realtime_tools::RealtimePublisher<sensor_msgs::JointState>* pub_joint_states_;
    uint64_t seq_nr_jnt_ = 0;
    uint64_t seq_nr_fra_ = 0;
    uint64_t missed_pulishes_franka_ = 0;
    uint64_t missed_pulishes_joint_ = 0;

    std::vector<std::string> joint_name_;  // joint_names

    std::array<double, 7> q_;  // robot state variables
    std::array<double, 7> dq_;
    std::array<double, 7> q_d_;
    std::array<double, 7> q_start_;
    std::array<double, 7> tau_J_;
    std::array<double, 7> dtau_J_;
    std::array<double, 7> tau_ext_hat_filtered_;
    std::array<double, 6> cartesian_collision_;
    std::array<double, 6> cartesian_contact_;
    std::array<double, 7> joint_collision_;
    std::array<double, 7> joint_contact_;
    std::array<double, 6> EE_F_ext_hat_EE_;
    std::array<double, 6> O_F_ext_hat_EE_;
    std::array<double, 2> elbow_start_;
    std::array<std::array<double, 4>, 4> O_T_EE_start_;
};

}  // namespace franka_hw
