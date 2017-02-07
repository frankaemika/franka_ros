#include <franka_hw/franka_hw.h>
#include <pluginlib/class_list_macros.h>
#include <franka/robot.h>
#include <string>

franka_hw::franka_robot::franka_robot()
{

}

franka_hw::franka_robot::~franka_robot()
{
    // TODO what to call to shutdown conntection to robot??
    delete robot_;
}

void franka_hw::franka_robot::init(ros::NodeHandle& nh)
{
    // parse robot_hw yaml with joint names and robot-IP from yaml
    XmlRpc::XmlRpcValue params;
    nh.getParam("joint_names", params);
    joint_name_.resize(params.size());
    for (int i = 0; i < params.size(); ++i)
    {
        joint_name_[i] = static_cast<std::string>(params[i]);
        ROS_INFO("joint %d: %s", i, joint_name_.at(i).c_str());
    }
    nh.getParam("robot_ip", robot_ip_);

    // setup Robot (needs ip to be parsed)
    setUpRobot();

    // resize members for state storage
    q_.resize(joint_name_.size());
    dq_.resize(joint_name_.size());
    q_d_.resize(joint_name_.size());
    q_start_.resize(joint_name_.size());
    tau_J_.resize(joint_name_.size());
    dtau_J_.resize(joint_name_.size());
    tau_ext_hat_filtered_.resize(joint_name_.size());
    cartesian_collision_.resize(6);
    cartesian_contact_.resize(6);
    joint_collision_.resize(joint_name_.size());
    joint_contact_.resize(joint_name_.size());
    EE_F_ext_hat_EE_.resize(6);
    O_F_ext_hat_EE_.resize(6);
    O_T_EE_start_.resize(16);
    elbow_start_.resize(2);

    // resize members for commands
    // TODO(Christoph): more commands??
    // cmd_q_d_.resize(joint_name_.size());
    // cmd_tau_J_d_.resize(joint_name_.size());
    // cmd_pose_d_.resize(6);  // TODO size 7 with quaternion instead of rpy?
    // ik_target_pose_.resize(6);  // TODO size 7 with quaternion instead of rpy?


    // register interfaces:

    for (size_t i = 0; i < joint_name_.size(); ++i)
    {
        // connect the standard joint state interface
        hardware_interface::JointStateHandle jnt_handle1(joint_name_[i], &q_[i], &dq_[i], &tau_J_[i]);
        jnt_state_interface_.registerHandle(jnt_handle1);

        // connect the franka joint state interface
        hardware_interface::FrankaJointStateHandle jnt_handle2(joint_name_[i], &q_[i], &dq_[i], &tau_J_[i],
                                                              &q_d_[i], &q_start_[i], &dtau_J_[i],
                                                              &tau_ext_hat_filtered_[i], &joint_collision_[i],
                                                              &joint_contact_[i]);
        franka_jnt_state_interface_.registerHandle(jnt_handle2);
    }
    registerInterface(&franka_jnt_state_interface_);
    registerInterface(&jnt_state_interface_);

    // TODO(Christoph) additional interfaces ????

}

bool franka_hw::franka_robot::read()
{
    try
    {
        // TODO(Christoph): read all necessary values into member variables
        return true;
    }
    catch(franka::NetworkException const& e)
    {
        std::cout << e.what() << std::endl << std::endl << std::endl << std::endl << std::endl << std::endl;
        return false;
    }
}

/* void franka_hw::franka_robot::write() const
{
    // TODO(Christoph): Offer different hardware interfaces for different control strategies, case sensitive write() command,
    // switch case??
}
*/

ros::Duration franka_hw::franka_robot::get_period() const
{
    return ros::Time::now() - prev_time_;
}

bool franka_hw::franka_robot::setUpRobot()
{
    try
    {
        robot_ = new franka::Robot(robot_ip_.c_str());
        return true;
    }
    catch(franka::NetworkException const& e)
    {
        std::cout << e.what() << std::endl << std::endl << std::endl << std::endl << std::endl << std::endl;
        return false;
    }
}

void franka_hw::franka_robot::setPrevTime(ros::Time time)
{
    prev_time_ = time;
}

std::string franka_hw::franka_robot::getRobotIp() const
{
    return robot_ip_;
}


PLUGINLIB_EXPORT_CLASS(franka_hw::franka_robot, hardware_interface::RobotHW)
