#include <franka_gazebo/franka_hw_sim.h>
#include <gazebo_ros_control/robot_hw_sim.h>

namespace franka_gazebo {

bool FrankaHWSim::initSim(const std::string& robot_namespace,
                          ros::NodeHandle model_nh,
                          gazebo::physics::ModelPtr parent,
                          const urdf::Model* const urdf,
                          std::vector<transmission_interface::TransmissionInfo> transmissions) {
  // Generate a list of franka_gazebo::Joint to store all relevant information
  auto dof = transmissions.size();
  this->joints_.resize(dof);
  for (int i = 0; i < dof; i++) {
    const auto& transmission = transmissions.at(i);
    if (transmission.joints_.size() == 0) {
      ROS_WARN_STREAM_NAMED("franka_hw_sim",
                            "Transmission " << transmission.name_ << " has no associated joints.");
      return false;
    }
    if (transmission.joints_.size() > 1) {
      ROS_WARN_STREAM_NAMED(
          "franka_hw_sim",
          "Transmission "
              << transmission.name_
              << " has more than one joint. Currently the franka robot hardware simulation "
              << " interface only supports one.");
      return false;
    }

    // Fill a 'Joint' struct which holds all necessary data
    franka_gazebo::Joint& joint = this->joints_[i];
    joint.name = transmission.joints_[0].name_;
    if (not urdf) {
      ROS_ERROR_STREAM_NAMED(
          "franka_hw_sim", "Could not find any URDF model. Was it loaded on the parameter server?");
      return false;
    }
    const auto urdfJoint = urdf->getJoint(joint.name);
    if (not urdfJoint) {
      ROS_ERROR_STREAM_NAMED("franka_hw_sim",
                             "Could not get joint '" << joint.name << "' from URDF");
      return false;
    }
    joint.type = urdfJoint->type;

    // Get a handle to the underlying Gazebo Joint
    gazebo::physics::JointPtr handle = parent->GetJoint(joint.name);
    if (not handle) {
      ROS_ERROR_STREAM_NAMED("franka_hw_sim", "This robot has a joint named '"
                                                  << joint.name
                                                  << "' which is not in the gazebo model.");
      return false;
    }
    joint.handle = handle;
  }

  // After the joint data containers have been fully initialized and their memory address don't
  // change anymore, get the respective addresses to pass them to the handles
  for (int i = 0; i < dof; i++) {
    auto& joint = this->joints_[i];

    // Register the state interface
    this->jsi_.registerHandle(hardware_interface::JointStateHandle(joint.name, &joint.position,
                                                                   &joint.velocity, &joint.effort));

    // Register all supported command interfaces
    bool frankaStateInterfaceFound = false;
    for (const auto interface : transmissions[i].joints_[0].hardware_interfaces_) {
      ROS_INFO_STREAM_NAMED("franka_hw_sim", "Found transmission interface of joint '"
                                                 << joint.name << "': " << interface);
      if (interface == "hardware_interface/EffortJointInterface") {
        this->eji_.registerHandle(
            hardware_interface::JointHandle(this->jsi_.getHandle(joint.name), &joint.command));
        continue;
      }

      if (interface == "franka_hw/FrankaStateInterface") {
        if (frankaStateInterfaceFound) continue;
        this->fsi_.registerHandle(
            franka_hw::FrankaStateHandle(robot_namespace + "_robot", this->robot_state_));
        frankaStateInterfaceFound = true;
        continue;
      }

      ROS_WARN_STREAM_NAMED("franka_hw_sim", "Unknown transmission interface of joint '"
                                                 << joint.name << "': " << interface);
    }
  }

  registerInterface(&this->jsi_);
  registerInterface(&this->eji_);
  registerInterface(&this->fsi_);

  return true;
}

void FrankaHWSim::readSim(ros::Time time, ros::Duration period) {
  for (franka_gazebo::Joint& joint : this->joints_) {
    joint.update();
  }
  this->robot_state_.m_ee = 42;
}

void FrankaHWSim::writeSim(ros::Time time, ros::Duration period) {}

void FrankaHWSim::eStopActive(bool active) {}

}  // namespace franka_gazebo

PLUGINLIB_EXPORT_CLASS(franka_gazebo::FrankaHWSim, gazebo_ros_control::RobotHWSim)
