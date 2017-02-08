#ifndef FRANKA_HW_FRANKA_HW_H
#define FRANKA_HW_FRANKA_HW_H

#include <pluginlib/class_list_macros.h>

#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include <franka_hw/franka_joint_state_interface.h>
#include <franka_hw/franka_cartesian_state_interface.h>

#include <franka/robot.h>

#include <vector>
#include <string>

namespace franka_hw
{

class FrankaHW: public hardware_interface::RobotHW
{
public:
    FrankaHW();
    ~FrankaHW();
    void init(const ros::NodeHandle& nh);
    bool update();
    void publishFrankaStates();
    void publishJointStates();
    void updateStates(const franka::RobotState& robot_state);
    ros::Duration get_period() const;
    bool setUpRobot(std::string ip);
    std::string getRobotIp() const;

private:
    hardware_interface::JointStateInterface jnt_state_interface_;  // interfaces
    hardware_interface::FrankaJointStateInterface franka_jnt_state_interface_;
    hardware_interface::FrankaCartesianStateInterface franka_cart_state_interface_;

    franka::Robot *robot_;  // libfranka robot

    std::vector<std::string> joint_name_;  // joint_names

    std::string robot_ip_;  // robot state variables
    std::vector<double> q_;
    std::vector<double> dq_;
    std::vector<double> q_d_;
    std::vector<double> q_start_;
    std::vector<double> tau_J_;
    std::vector<double> dtau_J_;
    std::vector<double> tau_ext_hat_filtered_;
    std::vector<double> cartesian_collision_;
    std::vector<double> cartesian_contact_;
    std::vector<double> joint_collision_;
    std::vector<double> joint_contact_;
    std::vector<double> EE_F_ext_hat_EE_;
    std::vector<double> O_F_ext_hat_EE_;
    std::vector<double> elbow_start_;
    std::vector<std::vector<double> > O_T_EE_start_;
};

}  // namespace franka_hw

#endif  // FRANKA_HW_FRANKA_HW_H
