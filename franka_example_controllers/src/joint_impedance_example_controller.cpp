#include <franka_example_controllers/joint_impedance_example_controller.h>

#include <cmath>

#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <xmlrpcpp/XmlRpcValue.h>

#include <franka/robot_state.h>

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

bool JointImpedanceExampleController::init(
    hardware_interface::RobotHW* robot_hw,
    ros::NodeHandle& node_handle) {
  if (!node_handle.getParam("arm_id", arm_id_)) {
    ROS_ERROR(
        "JointImpedanceExampleController: Could not read parameter arm_id");
    return false;
  }
  if (!node_handle.getParam("radius", radius_)) {
    ROS_INFO_STREAM(
        "JointImpedanceExampleController: No parameter radius, defaulting to: "
        << radius_);
  }
  if (std::fabs(radius_) < 0.005) {
    ROS_INFO_STREAM(
        "JointImpedanceExampleController: Set radius to small, defaulting to: "
        << 0.1);
    radius_ = 0.1;
  }

  if (!node_handle.getParam("vel_max", vel_max_)) {
    ROS_INFO_STREAM(
        "JointImpedanceExampleController: No parameter vel_max, defaulting to: "
        << vel_max_);
  }
  if (!node_handle.getParam("acceleration_time", acceleration_time_)) {
    ROS_INFO_STREAM(
        "JointImpedanceExampleController: No parameter acceleration_time, "
        "defaulting to: "
        << acceleration_time_);
  }

  XmlRpc::XmlRpcValue tmp;
  if (!node_handle.getParam("joint_names", tmp)) {
    ROS_ERROR(
        "JointImpedanceExampleController: Could not get joint_names, aborting "
        "controller init!");
    return false;
  }
  if (tmp.size() != 7) {
    ROS_ERROR(
        "JointImpedanceExampleController: Wrong number of joint_names, "
        "aborting controller init!");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    joint_names_.push_back(static_cast<std::string>(tmp[i]));
  }

  tmp.clear();
  if (!node_handle.getParam("k_gains", tmp)) {
    ROS_ERROR(
        "JointImpedanceExampleController: Could not get k_gains, aborting "
        "controller init!");
    return false;
  }
  if (tmp.size() != 7) {
    ROS_ERROR(
        "JointImpedanceExampleController: Wrong number of k_gains, aborting "
        "controller init!");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    k_gains_[i] = (static_cast<double>(tmp[i]));
  }

  tmp.clear();
  if (!node_handle.getParam("d_gains", tmp)) {
    ROS_ERROR(
        "JointImpedanceExampleController: Could not get d_gains, aborting "
        "controller init!");
    return false;
  }
  if (tmp.size() != 7) {
    ROS_ERROR(
        "JointImpedanceExampleController: Wrong number of d_gains, aborting "
        "controller init!");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    d_gains_[i] = (static_cast<double>(tmp[i]));
  }

  model_interface_ = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface_ == nullptr) {
    ROS_ERROR_STREAM(
        "JointImpedanceExampleController: Error getting model interface from "
        "hardware");
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

  cartesian_pose_interface_ =
      robot_hw->get<franka_hw::FrankaPoseCartesianInterface>();
  if (cartesian_pose_interface_ == nullptr) {
    ROS_ERROR_STREAM(
        "JointImpedanceExampleController: Error getting cartesian pose "
        "interface from hardware");
    return false;
  }
  try {
    cartesian_pose_handle_.reset(new franka_hw::FrankaCartesianPoseHandle(
        cartesian_pose_interface_->getHandle(arm_id_ + "_robot")));
  } catch (hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "JointImpedanceExampleController: Exception getting cartesian pose "
        "handle from "
        "interface: "
        << e.what());
    return false;
  }
  initial_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE;

  effort_joint_interface_ =
      robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface_ == nullptr) {
    ROS_ERROR_STREAM(
        "JointImpedanceExampleController: Error getting effort joint interface "
        "from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(
          effort_joint_interface_->getHandle(joint_names_[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "JointImpedanceExampleController: Exception getting joint handles: "
          << ex.what());
      return false;
    }
  }

  ROS_INFO(
      "JointImpedanceExampleController: Finished initializing Joint Impedance "
      "Controller ---------------");
  return true;
}

void JointImpedanceExampleController::update(const ros::Time& /*time*/,
                                             const ros::Duration& period) {
  ROS_INFO("JointImpedanceExampleController: Control Loop");
  if (vel_current_ < vel_max_) {
    vel_current_ += period.toSec() * std::fabs(vel_max_ / acceleration_time_);
  }
  ROS_INFO("JointImpedanceExampleController: DB1");
  vel_current_ = std::fmin(vel_current_, vel_max_);

  angle_ += period.toSec() * vel_current_ / std::fabs(radius_);
  if (angle_ > 2 * M_PI) {
    angle_ -= 2 * M_PI;
  }

  ROS_INFO("JointImpedanceExampleController: DB2");
  double delta_y = radius_ * (1 - std::cos(angle_));
  double delta_z = radius_ * std::sin(angle_);

  std::array<double, 16> pose_desired = initial_pose_;
  pose_desired[13] += delta_y;
  pose_desired[14] += delta_z;
  cartesian_pose_handle_->setCommand(pose_desired);

  ROS_INFO("JointImpedanceExampleController: DB3");
  franka::RobotState robot_state = cartesian_pose_handle_->getRobotState();
  std::array<double, 7> q_desired = robot_state.q_d;
  std::array<double, 7> q_current = robot_state.q;
  std::array<double, 7> dq = robot_state.dq;
  std::array<double, 7> coriolis = model_handle_->getCoriolis(
      {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}}, 0.0, {{0.0, 0.0, 0.0}});
  std::array<double, 7> gravity =
      model_handle_->getGravity(0.0, {{0.0, 0.0, 0.0}});
  ROS_INFO("JointImpedanceExampleController: DB4");

  std::array<double, 7> tau_d;
  for (size_t i = 0; i < 7; ++i) {
    tau_d[i] = k_gains_[i] * (q_desired[i] - q_current[i]) -
               std::fabs(d_gains_[i]) * dq[i] + coriolis[i];
    joint_handles_[i].setCommand(tau_d[i]);
  }
  ROS_INFO("JointImpedanceExampleController: DB5");

  if (rate_trigger_()) {
    std::array<double, 7> tau_j = robot_state.tau_J;
    std::array<double, 7> tau_diff;
    for (size_t i = 0; i < 7; ++i) {
      tau_diff[i] = last_tau_d_[i] - tau_j[i];
      ROS_INFO(
          "--------------------------------------------------------------------"
          "-----------");
      ROS_INFO_STREAM("tau_diff: " << tau_diff << " last_tau_d: " << last_tau_d_
                                   << " tau_j: " << tau_j);
    }
  }

  ROS_INFO("JointImpedanceExampleController: DB6");
  for (size_t i = 0; i < 7; ++i) {
    last_tau_d_[i] = tau_d[i] + gravity[i];
  }
  ROS_INFO("Finishing control loop");
  return;
}

void JointImpedanceExampleController::stopping(const ros::Time& /*time*/) {
  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(0.0);
  }
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(
    franka_example_controllers::JointImpedanceExampleController,
    controller_interface::ControllerBase)
