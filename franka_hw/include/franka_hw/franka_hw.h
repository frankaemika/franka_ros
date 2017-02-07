#ifndef FRANKA_HW_FRANKA_HW_H
#define FRANKA_HW_FRANKA_HW_H

#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include <franka_hw/franka_joint_state_interface.h>
#include <franka_hw/franka_cartesian_state_interface.h>

#include <franka/robot.h>

#include <vector>
#include <string>

namespace franka_hw
{

enum controlStrategy   // TODO update with implemented strategies!!!
{
    cartesianImpedanceControl = 1,
    cartesianPosition = 2,
    cartesianVelocity = 3,
    jointTorqueControl = 4,
    jointImpedanceControl  = 5,
    jointPosition = 6,
    jointVelocity = 7
};


class franka_robot: public hardware_interface::RobotHW
{
public:
    franka_robot();
    ~franka_robot();
    void init(ros::NodeHandle& nh);
    bool read();
//  void write() const;
    ros::Duration get_period() const;
    bool setUpRobot();
    void setPrevTime(ros::Time time);
    std::string getRobotIp() const;

private:
    hardware_interface::JointStateInterface jnt_state_interface_;
    hardware_interface::FrankaJointStateInterface franka_jnt_state_interface_;
    hardware_interface::FrankaCartesianStateInterface franka_cart_state_interface_;

    std::vector<std::string> joint_name_;

    // robot state variables:
    std::string robot_ip_;
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
    std::vector<double> O_T_EE_start_;
    std::vector<double> elbow_start_;

    // TODO(Christoph): more commands??
    // robot command variables
    // std::vector<double> cmd_q_d_;
    // std::vector<double> cmd_tau_J_d_;
    // std::vector<double> cmd_pose_d_;
    // std::vector<double> ik_target_pose_;

    ros::Time prev_time_;
    franka::Robot *robot_;
    // controlStrategy ctrl_strategy_;
};

}  // namespace franka_hw
#endif  // FRANKA_HW_FRANKA_HW_H
