#include <franka_hw/franka_hw.h>

#include <pluginlib/class_list_macros.h>
#include <array>
#include <cinttypes>
#include <mutex>
#include <string>

#include <franka/robot.h>
#include <realtime_tools/realtime_publisher.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/MultiArrayDimension.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf2_msgs/TFMessage.h>

#include <franka_hw/FrankaState.h>

namespace franka_hw {

// A default constructor is required to export the plugin with pluginlib
FrankaHW::FrankaHW() : robot_("0.0.0.0") {}

FrankaHW::FrankaHW(const std::vector<std::string>& joint_names,
                   const std::string& ip,
                   double publish_rate,
                   const ros::NodeHandle& nh)
    : joint_state_interface_(),
      franka_joint_state_interface_(),
      franka_cartesian_state_interface_(),
      publish_rate_(publish_rate),
      robot_(ip),
      publisher_transforms_(nh, "/tf", 1),
      publisher_franka_states_(nh, "franka_states", 1),
      publisher_joint_states_(nh, "joint_states", 1),
      publisher_external_wrench_(nh, "F_ext", 1),
      joint_name_(joint_names),
      robot_state_() {
  for (size_t i = 0; i < joint_name_.size(); ++i) {
    hardware_interface::JointStateHandle joint_handle(
        joint_name_[i], &robot_state_.q[i], &robot_state_.dq[i],
        &robot_state_.tau_J[i]);
    joint_state_interface_.registerHandle(joint_handle);
    franka_hw::FrankaJointStateHandle franka_joint_handle(
        joint_name_[i], robot_state_.q[i], robot_state_.dq[i],
        robot_state_.tau_J[i], robot_state_.q_d[i], robot_state_.dtau_J[i],
        robot_state_.tau_ext_hat_filtered[i], robot_state_.joint_collision[i],
        robot_state_.joint_contact[i]);
    franka_joint_state_interface_.registerHandle(franka_joint_handle);
  }

  FrankaCartesianStateHandle franka_cartesian_handle(
      std::string("franka_emika_cartesian_data"),
      robot_state_.cartesian_collision, robot_state_.cartesian_contact,
      robot_state_.O_F_ext_hat_K, robot_state_.K_F_ext_hat_K,
      robot_state_.O_T_EE);
  franka_cartesian_state_interface_.registerHandle(franka_cartesian_handle);

  registerInterface(&joint_state_interface_);
  registerInterface(&franka_joint_state_interface_);
  registerInterface(&franka_cartesian_state_interface_);

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
    publisher_joint_states_.msg_.name.resize(joint_name_.size());
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
    robot_.read([this, callback](const franka::RobotState& robot_state) {
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
    for (size_t i = 0; i < joint_name_.size(); ++i) {
      publisher_joint_states_.msg_.name[i] = joint_name_[i];
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

}  // namespace franka_hw

PLUGINLIB_EXPORT_CLASS(franka_hw::FrankaHW, hardware_interface::RobotHW)
