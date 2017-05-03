#include <franka_hw/franka_hw.h>

#include <pluginlib/class_list_macros.h>
#include <array>
#include <cinttypes>
#include <mutex>
#include <string>

#include <franka/robot.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <realtime_tools/realtime_publisher.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/MultiArrayDimension.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf2_msgs/TFMessage.h>
#include <urdf/model.h>

#include <franka_hw/FrankaState.h>

namespace franka_hw {

// A default constructor is required to export the plugin with pluginlib
FrankaHW::FrankaHW() {}

FrankaHW::FrankaHW(const std::vector<std::string>& joint_names,
                   const std::string& ip,
                   double publish_rate,
                   const ros::NodeHandle& node_handle)
    : joint_state_interface_(),
      franka_joint_state_interface_(),
      franka_cartesian_state_interface_(),
      position_joint_interface_(),
      velocity_joint_interface_(),
      effort_joint_interface_(),
      franka_pose_cartesian_interface_(),
      franka_velocity_cartesian_interface_(),
      position_joint_limit_interface_(),
      velocity_joint_limit_interface_(),
      effort_joint_limit_interface_(),
      publish_rate_(),
      robot_(new franka::Robot(ip)),
      publisher_transforms_(node_handle, "/tf", 1),
      publisher_franka_states_(node_handle, "franka_states", 1),
      publisher_joint_states_(node_handle, "joint_states", 1),
      publisher_external_wrench_(node_handle, "F_ext", 1),
      joint_names_(),
      robot_state_() {
  initialize(joint_names, publish_rate, node_handle);
}

void FrankaHW::initialize(const std::vector<std::string>& joint_names,
                          double publish_rate,
                          const ros::NodeHandle& node_handle) {
  joint_names_.resize(joint_names.size());
  joint_names_ = joint_names;
  publish_rate_.setRate(publish_rate);
  urdf::Model urdf_model;
  if (!urdf_model.initParamWithNodeHandle("robot_description", node_handle)) {
      ROS_ERROR("Could not initialize urdf model from robot_description");
  }
  joint_limits_interface::SoftJointLimits soft_limits;
  joint_limits_interface::JointLimits limits;

  for (size_t i = 0; i < joint_names_.size(); ++i) {
    hardware_interface::JointStateHandle joint_handle(
        joint_names_[i], &robot_state_.q[i], &robot_state_.dq[i],
        &robot_state_.tau_J[i]);
    joint_state_interface_.registerHandle(joint_handle);

    franka_hw::FrankaJointStateHandle franka_joint_handle(
        joint_names_[i], robot_state_.q[i], robot_state_.dq[i],
        robot_state_.tau_J[i], robot_state_.q_d[i], robot_state_.dtau_J[i],
        robot_state_.tau_ext_hat_filtered[i], robot_state_.joint_collision[i],
        robot_state_.joint_contact[i]);
    franka_joint_state_interface_.registerHandle(franka_joint_handle);

    hardware_interface::JointHandle position_joint_handle(
        joint_state_interface_.getHandle(joint_names_[i]),
        &position_joint_command_[i]);
    position_joint_interface_.registerHandle(position_joint_handle);

    hardware_interface::JointHandle velocity_joint_handle(
        joint_state_interface_.getHandle(joint_names_[i]),
        &velocity_joint_command_[i]);
    velocity_joint_interface_.registerHandle(velocity_joint_handle);

    hardware_interface::JointHandle effort_joint_handle(
        joint_state_interface_.getHandle(joint_names_[i]),
        &effort_joint_command_[i]);
    effort_joint_interface_.registerHandle(effort_joint_handle);

    boost::shared_ptr<const urdf::Joint> urdf_joint =
        urdf_model.getJoint(joint_names_[i]);
    if (!urdf_joint) {
        ROS_ERROR_STREAM("Could not get joint " << joint_names_[i] << " from urdf");
    }
    if (!urdf_joint->safety) {
        ROS_ERROR_STREAM("Joint " << joint_names_[i] << " has no safety");
    }
    if (!urdf_joint->limits) {
        ROS_ERROR_STREAM("Joint " << joint_names_[i] << " has no limits");
    }

    if (joint_limits_interface::getSoftJointLimits(urdf_joint, soft_limits)) {

        if (joint_limits_interface::getJointLimits(urdf_joint, limits)) {
            joint_limits_interface::PositionJointSoftLimitsHandle
                    position_limit_handle(
                        position_joint_interface_.getHandle(joint_names_[i]), limits,
                        soft_limits);
            position_joint_limit_interface_.registerHandle(position_limit_handle);

            joint_limits_interface::VelocityJointSoftLimitsHandle
                    velocity_limit_handle(
                        velocity_joint_interface_.getHandle(joint_names_[i]), limits,
                        soft_limits);
            velocity_joint_limit_interface_.registerHandle(velocity_limit_handle);

            joint_limits_interface::EffortJointSoftLimitsHandle effort_limit_handle(
                        effort_joint_interface_.getHandle(joint_names_[i]), limits,
                        soft_limits);
            effort_joint_limit_interface_.registerHandle(effort_limit_handle);
        } else {
            ROS_ERROR_STREAM("Could not parse joint limit for joint "
                             << joint_names_[i] << " for joint limit interfaces");
        }
    } else {
        ROS_ERROR_STREAM("Could not parse soft joint limit for joint "
                         << joint_names_[i] << " for joint limit interfaces");
    }
  }

  FrankaCartesianStateHandle franka_cartesian_state_handle(
      std::string("franka_cartesian_data"), robot_state_.cartesian_collision,
      robot_state_.cartesian_contact, robot_state_.O_F_ext_hat_K,
      robot_state_.K_F_ext_hat_K, robot_state_.O_T_EE);
  franka_cartesian_state_interface_.registerHandle(
      franka_cartesian_state_handle);

  franka_hw::FrankaCartesianPoseHandle franka_cartesian_pose_handle(
      franka_cartesian_state_interface_.getHandle("franka_cartesian_data"),
      pose_cartesian_command_);
  franka_pose_cartesian_interface_.registerHandle(franka_cartesian_pose_handle);

  franka_hw::FrankaCartesianVelocityHandle franka_cartesian_velocity_handle(
      franka_cartesian_state_interface_.getHandle("franka_cartesian_data"),
      velocity_cartesian_command_);
  franka_velocity_cartesian_interface_.registerHandle(
      franka_cartesian_velocity_handle);

  registerInterface(&joint_state_interface_);
  registerInterface(&franka_joint_state_interface_);
  registerInterface(&franka_cartesian_state_interface_);
  registerInterface(&position_joint_interface_);
  registerInterface(&velocity_joint_interface_);
  registerInterface(&effort_joint_interface_);
  registerInterface(&franka_pose_cartesian_interface_);
  registerInterface(&franka_velocity_cartesian_interface_);

  {
    std::lock_guard<realtime_tools::RealtimePublisher<franka_hw::FrankaState> >
        lock(publisher_franka_states_);
    publisher_franka_states_.msg_.cartesian_collision.resize(
        robot_state_.cartesian_collision.size());
    publisher_franka_states_.msg_.cartesian_contact.resize(
        robot_state_.cartesian_contact.size());
    publisher_franka_states_.msg_.dq.resize(robot_state_.dq.size());
    publisher_franka_states_.msg_.dtau_J.resize(robot_state_.dtau_J.size());
    publisher_franka_states_.msg_.K_F_ext_hat_K.resize(
        robot_state_.K_F_ext_hat_K.size());
    publisher_franka_states_.msg_.elbow.resize(robot_state_.elbow.size());
    publisher_franka_states_.msg_.joint_collision.resize(
        robot_state_.joint_collision.size());
    publisher_franka_states_.msg_.joint_contact.resize(
        robot_state_.joint_contact.size());
    publisher_franka_states_.msg_.O_F_ext_hat_K.resize(
        robot_state_.O_F_ext_hat_K.size());
    publisher_franka_states_.msg_.q.resize(robot_state_.q.size());
    publisher_franka_states_.msg_.q_d.resize(robot_state_.q_d.size());
    publisher_franka_states_.msg_.tau_ext_hat_filtered.resize(
        robot_state_.tau_ext_hat_filtered.size());
    publisher_franka_states_.msg_.tau_J.resize(robot_state_.tau_J.size());
    publisher_franka_states_.msg_.O_T_EE.layout.data_offset = 0;
    publisher_franka_states_.msg_.O_T_EE.layout.dim.clear();
    publisher_franka_states_.msg_.O_T_EE.layout.dim.push_back(
        std_msgs::MultiArrayDimension());
    publisher_franka_states_.msg_.O_T_EE.layout.dim[0].size = 4;
    publisher_franka_states_.msg_.O_T_EE.layout.dim[0].stride = 4 * 4;
    publisher_franka_states_.msg_.O_T_EE.layout.dim[0].label = "row";
    publisher_franka_states_.msg_.O_T_EE.layout.dim.push_back(
        std_msgs::MultiArrayDimension());
    publisher_franka_states_.msg_.O_T_EE.layout.dim[1].size = 4;
    publisher_franka_states_.msg_.O_T_EE.layout.dim[1].stride = 4;
    publisher_franka_states_.msg_.O_T_EE.layout.dim[1].label = "column";
    publisher_franka_states_.msg_.O_T_EE.data.resize(16);
  }
  {
    std::lock_guard<realtime_tools::RealtimePublisher<sensor_msgs::JointState> >
        lock(publisher_joint_states_);
    publisher_joint_states_.msg_.name.resize(joint_names_.size());
    publisher_joint_states_.msg_.position.resize(robot_state_.q.size());
    publisher_joint_states_.msg_.velocity.resize(robot_state_.dq.size());
    publisher_joint_states_.msg_.effort.resize(robot_state_.tau_J.size());
  }
  {
    std::lock_guard<realtime_tools::RealtimePublisher<tf2_msgs::TFMessage> >
        lock(publisher_transforms_);
    publisher_transforms_.msg_.transforms.resize(2);
    tf::Quaternion quaternion(0.0, 0.0, 0.0, 1.0);
    tf::Vector3 translation(0.0, 0.0, 0.05);
    tf::Transform transform(quaternion, translation);
    tf::StampedTransform trafo(transform, ros::Time::now(), "link8", "EE");
    geometry_msgs::TransformStamped transform_message;
    transformStampedTFToMsg(trafo, transform_message);
    publisher_transforms_.msg_.transforms[0] = transform_message;
    translation = tf::Vector3(0.0, 0.0, 0.0);
    transform = tf::Transform(quaternion, translation);
    trafo = tf::StampedTransform(transform, ros::Time::now(), "EE", "K");
    transformStampedTFToMsg(trafo, transform_message);
    publisher_transforms_.msg_.transforms[1] = transform_message;
  }
  {
    std::lock_guard<
        realtime_tools::RealtimePublisher<geometry_msgs::WrenchStamped> >
        lock(publisher_external_wrench_);
    publisher_external_wrench_.msg_.header.frame_id = "K";
    publisher_external_wrench_.msg_.wrench.force.x = 0.0;
    publisher_external_wrench_.msg_.wrench.force.y = 0.0;
    publisher_external_wrench_.msg_.wrench.force.z = 0.0;
    publisher_external_wrench_.msg_.wrench.torque.x = 0.0;
    publisher_external_wrench_.msg_.wrench.torque.y = 0.0;
    publisher_external_wrench_.msg_.wrench.torque.z = 0.0;
  }
}

bool franka_hw::FrankaHW::update(std::function<bool(const franka::RobotState&)> callback) {
  try {
    robot_->read([this, callback](const franka::RobotState& robot_state) {
      robot_state_ = robot_state;
      if (publish_rate_.triggers()) {
        publishFrankaStates();
        publishJointStates();
        publishTransforms();
        publishExternalWrench();
      }
      return callback(robot_state);
    });
  } catch (const franka::Exception& e) {
    ROS_ERROR_STREAM("" << e.what());
    return false;
  }
  return true;
}

void FrankaHW::publishFrankaStates() {
  if (publisher_franka_states_.trylock()) {
    for (size_t i = 0; i < robot_state_.cartesian_collision.size(); ++i) {
      publisher_franka_states_.msg_.cartesian_collision[i] =
          robot_state_.cartesian_collision[i];
      publisher_franka_states_.msg_.cartesian_contact[i] =
          robot_state_.cartesian_contact[i];
      publisher_franka_states_.msg_.K_F_ext_hat_K[i] =
          robot_state_.K_F_ext_hat_K[i];
      publisher_franka_states_.msg_.O_F_ext_hat_K[i] =
          robot_state_.O_F_ext_hat_K[i];
    }

    for (size_t i = 0; i < robot_state_.q.size(); ++i) {
      publisher_franka_states_.msg_.q[i] = robot_state_.q[i];
      publisher_franka_states_.msg_.dq[i] = robot_state_.dq[i];
      publisher_franka_states_.msg_.tau_J[i] = robot_state_.tau_J[i];
      publisher_franka_states_.msg_.dtau_J[i] = robot_state_.dtau_J[i];
      publisher_franka_states_.msg_.joint_collision[i] =
          robot_state_.joint_collision[i];
      publisher_franka_states_.msg_.joint_contact[i] =
          robot_state_.joint_contact[i];
      publisher_franka_states_.msg_.q_d[i] = robot_state_.q_d[i];
      publisher_franka_states_.msg_.tau_ext_hat_filtered[i] =
          robot_state_.tau_ext_hat_filtered[i];
    }

    for (size_t i = 0; i < robot_state_.elbow.size(); ++i) {
      publisher_franka_states_.msg_.elbow[i] = robot_state_.elbow[i];
    }

    for (size_t row = 0; row < 4; ++row) {
      for (size_t col = 0; col < 4; ++col) {
        publisher_franka_states_.msg_.O_T_EE.data[4 * row + col] =
            robot_state_.O_T_EE[4 * col + row];
      }
    }

    publisher_franka_states_.msg_.header.seq = sequence_number_franka_states_;
    publisher_franka_states_.msg_.header.stamp = ros::Time::now();
    publisher_franka_states_.unlockAndPublish();
    sequence_number_franka_states_++;
  } else {
    sequence_number_franka_states_++;
  }
}

void FrankaHW::publishJointStates() {
  if (publisher_joint_states_.trylock()) {
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      publisher_joint_states_.msg_.name[i] = joint_names_[i];
      publisher_joint_states_.msg_.position[i] = robot_state_.q[i];
      publisher_joint_states_.msg_.velocity[i] = robot_state_.dq[i];
      publisher_joint_states_.msg_.effort[i] = robot_state_.tau_J[i];
    }
    publisher_joint_states_.msg_.header.stamp = ros::Time::now();
    publisher_joint_states_.msg_.header.seq = sequence_number_joint_states_;
    publisher_joint_states_.unlockAndPublish();
    sequence_number_joint_states_++;
  } else {
    sequence_number_joint_states_++;
  }
}

void FrankaHW::publishTransforms() {
  if (publisher_transforms_.trylock()) {
    tf::Quaternion quaternion(0.0, 0.0, 0.0, 1.0);
    tf::Vector3 translation(0.0, 0.0, 0.05);
    tf::Transform transform(quaternion, translation);
    tf::StampedTransform trafo(transform, ros::Time::now(), "link8", "EE");
    geometry_msgs::TransformStamped transform_message;
    transformStampedTFToMsg(trafo, transform_message);
    publisher_transforms_.msg_.transforms[0] = transform_message;
    translation = tf::Vector3(0.0, 0.0, 0.0);
    transform = tf::Transform(quaternion, translation);
    trafo = tf::StampedTransform(transform, ros::Time::now(), "EE", "K");
    transformStampedTFToMsg(trafo, transform_message);
    publisher_transforms_.msg_.transforms[1] = transform_message;
    publisher_transforms_.unlockAndPublish();
  }
}

void FrankaHW::publishExternalWrench() {
  if (publisher_external_wrench_.trylock()) {
    publisher_external_wrench_.msg_.header.frame_id = "K";
    publisher_external_wrench_.msg_.wrench.force.x =
        robot_state_.K_F_ext_hat_K[0];
    publisher_external_wrench_.msg_.wrench.force.y =
        robot_state_.K_F_ext_hat_K[1];
    publisher_external_wrench_.msg_.wrench.force.z =
        robot_state_.K_F_ext_hat_K[2];
    publisher_external_wrench_.msg_.wrench.torque.x =
        robot_state_.K_F_ext_hat_K[3];
    publisher_external_wrench_.msg_.wrench.torque.y =
        robot_state_.K_F_ext_hat_K[4];
    publisher_external_wrench_.msg_.wrench.torque.z =
        robot_state_.K_F_ext_hat_K[5];
    publisher_external_wrench_.unlockAndPublish();
  }
}

void FrankaHW::enforceLimits(const ros::Duration period) {
        position_joint_limit_interface_.enforceLimits(period);
        velocity_joint_limit_interface_.enforceLimits(period);
        effort_joint_limit_interface_.enforceLimits(period);
}

std::array<double, 7> FrankaHW::getJointPositionCommand() const {
    return position_joint_command_;
}

std::array<double, 7> FrankaHW::getJointVelocityCommand() const {
    return velocity_joint_command_;
}

std::array<double, 7> FrankaHW::getJointEffortCommand() const {
    return effort_joint_command_;
}

bool FrankaHW::checkForConflict(const std::list<hardware_interface::ControllerInfo> &info) const {
    return false; // Will be replaced later by checks for compatible interfaces
}

}  // namespace franka_hw

PLUGINLIB_EXPORT_CLASS(franka_hw::FrankaHW, hardware_interface::RobotHW)
