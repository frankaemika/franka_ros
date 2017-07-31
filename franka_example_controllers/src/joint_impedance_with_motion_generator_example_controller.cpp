#include <franka_example_controllers/joint_impedance_with_motion_generator_example_controller.h>

#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <xmlrpcpp/XmlRpcValue.h>


namespace {
template <class T, size_t N>
std::ostream& operator<<(std::ostream& ostream, const std::array<T, N>& array) {
  ostream << "[";
  std::copy(array.cbegin(), array.cend() - 1,
            std::ostream_iterator<T>(ostream, ","));
  std::copy(array.cend() - 1, array.cend(), std::ostream_iterator<T>(ostream));
  ostream << "]";
  return ostream;
}
}  // anonymous namespace

namespace franka_example_controllers {

JointImpedanceExampleController::JointImpedanceExampleController()
    : rate_trigger_(1.0) {}

bool JointImpedanceExampleController::init(hardware_interface::RobotHW* robot_hw,
                                  ros::NodeHandle& node_handle) {
  if (!node_handle.getParam("arm_id", arm_id_)) {
    ROS_ERROR("JointImpedanceExampleController: Could not read parameter arm_id");
    return false;
  }
  if (!node_handle.getParam("radius", radius_)) {
    ROS_INFO_STREAM("JointImpedanceExampleController: No parameter radius, defaulting to: " << radius_);
  }
  if (!node_handle.getParam("vel_max", vel_max_)) {
    ROS_INFO_STREAM("JointImpedanceExampleController: No parameter vel_max, defaulting to: " << vel_max_);
  }
  if (!node_handle.getParam("acceleration_time", acceleration_time_)) {
    ROS_INFO_STREAM("JointImpedanceExampleController: No parameter acceleration_time, defaulting to: " << acceleration_time_);
  }

  XmlRpc::XmlRpcValue tmp;
  if (!node_handle.getParam("joint_names", tmp)) {
      ROS_ERROR("JointImpedanceExampleController: Could not get joint_names, aborting controller init!");
      return false;
  }
  if (tmp.size() !=7) {
      ROS_ERROR("JointImpedanceExampleController: Wrong number of joint_names, aborting controller init!");
      return false;
  }
  for(size_t i = 0; i < 7; ++i) {
      joint_names_.push_back(static_cast<std::string>(tmp[i]));
  }

  tmp.clear();
  if (!node_handle.getParam("k_gains", tmp)) {
      ROS_ERROR("JointImpedanceExampleController: Could not get k_gains, aborting controller init!");
      return false;
  }
  if (tmp.size() !=7) {
      ROS_ERROR("JointImpedanceExampleController: Wrong number of k_gains, aborting controller init!");
      return false;
  }
  for(size_t i = 0; i < 7; ++i) {
      k_gains_[i] = (static_cast<double>(tmp[i]));
  }

  tmp.clear();
  if (!node_handle.getParam("d_gains", tmp)) {
      ROS_ERROR("JointImpedanceExampleController: Could not get d_gains, aborting controller init!");
      return false;
  }
  if (tmp.size() !=7) {
      ROS_ERROR("JointImpedanceExampleController: Wrong number of d_gains, aborting controller init!");
      return false;
  }
  for (size_t i = 0; i < 7; ++i) {
      d_gains_[i] = (static_cast<double>(tmp[i]));
  }

  model_interface_ = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface_ == nullptr) {
    ROS_ERROR_STREAM(
        "JointImpedanceExampleController: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_.reset(new franka_hw::FrankaModelHandle(
        model_interface_->getHandle(arm_id_ + "_model")));
  } catch (hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "JointImpedanceExampleController: Exception getting model handle from "
        "interface: "
        << e.what());
    return false;
  }

  cartesian_pose_interface_ = robot_hw->get<franka_hw::FrankaPoseCartesianInterface>();
  if (cartesian_pose_interface_ == nullptr) {
    ROS_ERROR_STREAM(
        "JointImpedanceExampleController: Error getting cartesian pose interface from hardware");
    return false;
  }
  try {
    cartesian_pose_handle_.reset(new franka_hw::FrankaCartesianPoseHandle(
        cartesian_pose_interface_->getHandle(arm_id_ + "_robot")));
  } catch (hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "JointImpedanceExampleController: Exception getting cartesian pose handle from "
        "interface: "
        << e.what());
    return false;
  }

  effort_joint_interface_ = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface_ == nullptr) {
    ROS_ERROR_STREAM(
        "JointImpedanceExampleController: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
      try {
        joint_handles_.push_back(effort_joint_interface_->getHandle(joint_names_[i]));
      } catch (const hardware_interface::HardwareInterfaceException& ex) {
        ROS_ERROR_STREAM(
            "JointImpedanceExampleController: Exception getting joint handles: "
            << ex.what());
        return false;
      }
  }

  return true;
}

void JointImpedanceExampleController::update(const ros::Time& /*time*/,
                                    const ros::Duration& period) {

  // TODO

  if (rate_trigger_()) {
    // compare and print
  }
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::JointImpedanceExampleController,
                       controller_interface::ControllerBase)
