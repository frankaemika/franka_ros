
#include <franka_example_controllers/joint_velocity_controller_example.h>

#include <cmath>

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <xmlrpcpp/XmlRpcValue.h>


namespace franka_example_controllers {

JointVelocityExampleController::JointVelocityExampleController() : velocity_joint_interface_(nullptr) {
}

bool JointVelocityExampleController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle &node_handle) {
    velocity_joint_interface_ = robot_hw->get<hardware_interface::VelocityJointInterface>();
    if (velocity_joint_interface_ == nullptr) {
        ROS_ERROR("Error getting velocity joint interface from harware!");
        return false;
    }
    XmlRpc::XmlRpcValue parameters;
    if (!node_handle.getParam("/franka_hw_node/joint_names", parameters)) {
        ROS_ERROR("Could not parse joint names in JointLimitTestController");
    }
    if (parameters.size() != 7) {
        ROS_ERROR_STREAM("Wrong number of joint names, got "
                         << int(parameters.size())
                         << " instead of 7 names!");
        return false;
    }
    velocity_joint_handles_.resize(7);
    joint_names_.resize(7);
    for (size_t i = 0; i < 7; ++i) {
        joint_names_[i] = static_cast<std::string>(parameters[i]);
        try {
          velocity_joint_handles_[i] = velocity_joint_interface_->getHandle(joint_names_[i]);
        } catch (const hardware_interface::HardwareInterfaceException& e) {
            ROS_ERROR_STREAM("Exception getting joint handles: " << e.what());
            return false;
        }
    }
    start_time_stamp_=ros::Time::now();
    return true;
}

void JointVelocityExampleController::update(const ros::Time& time, const ros::Duration& period) {
    ros::Duration time_max(4.0);
    ros::Duration elapsed_time = ros::Time::now() - start_time_stamp_;
    double omega_max = 0.2;
              double cycle = std::floor(
              std::pow(-1.0, (elapsed_time.toSec() - std::fmod(elapsed_time.toSec(),
                              time_max.toSec())) / time_max.toSec()));
          double omega = cycle * omega_max / 2.0 *
                         (1.0 - std::cos(2.0 * M_PI / time_max.toSec() * elapsed_time.toSec()));
    for (size_t i = 0; i < 3; ++i) {
        velocity_joint_handles_[i].setCommand(0.0);
    }
    for (size_t i = 3; i < 7; ++i) {
        velocity_joint_handles_[i].setCommand(omega);
    }
}

void JointVelocityExampleController::stopping(const ros::Time& time) {
   for (size_t i = 0; i < velocity_joint_handles_.size(); ++i) {
       velocity_joint_handles_[i].setCommand(0.0);
   }
}

}  // franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::JointVelocityExampleController, controller_interface::ControllerBase)
