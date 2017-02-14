#include <franka_hw/franka_hw.h>

#include <math.h>  // floor()
#include <pluginlib/class_list_macros.h>
#include <array>
#include <string>

#include <franka/robot.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/MultiArrayDimension.h>

#include <franka_hw/FrankaState.h>

// apparently a constructor without inputs is required to export the plugin with
// pluginlib..
franka_hw::FrankaHW::FrankaHW() : robot_("0.0.0.0") {}

franka_hw::FrankaHW::~FrankaHW() {}

franka_hw::FrankaHW::FrankaHW(const std::vector<std::string>& joint_names,
                              const std::string& ip,
                              const ros::NodeHandle& nh)
    : robot_(ip.c_str()),
      joint_name_(joint_names),
      publisher_franka_states_(nh, "franka_states", 1),
      publisher_joint_states_(nh, "joint_states", 1)

{
  for (size_t i = 0; i < joint_name_.size(); ++i) {
    hardware_interface::JointStateHandle joint_handle1(
        joint_name_[i], &robot_state_.q[i], &robot_state_.dq[i],
        &robot_state_.tau_J[i]);
    joint_state_interface_.registerHandle(joint_handle1);
    hardware_interface::FrankaJointStateHandle joint_handle2(
        joint_name_[i], robot_state_.q[i], robot_state_.dq[i],
        robot_state_.tau_J[i], robot_state_.q_d[i], robot_state_.q_start[i],
        robot_state_.dtau_J[i], robot_state_.tau_ext_hat_filtered[i],
        robot_state_.joint_collision[i], robot_state_.joint_contact[i]);
    franka_joint_state_interface_.registerHandle(joint_handle2);
  }

  hardware_interface::FrankaCartesianStateHandle cartesian_handle(
      std::string("franka_emika_cartesian_data"),
      robot_state_.cartesian_collision, robot_state_.cartesian_contact,
      robot_state_.O_F_ext_hat_EE, robot_state_.EE_F_ext_hat_EE,
      robot_state_.O_T_EE_start);
  franka_cartesian_state_interface_.registerHandle(cartesian_handle);

  registerInterface(&joint_state_interface_);
  registerInterface(&franka_joint_state_interface_);
  registerInterface(&franka_cartesian_state_interface_);

  if (publisher_franka_states_.trylock()) {
    publisher_franka_states_.msg_.cartesian_collision.resize(
        robot_state_.cartesian_collision.size());
    publisher_franka_states_.msg_.cartesian_contact.resize(
        robot_state_.cartesian_contact.size());
    publisher_franka_states_.msg_.dq.resize(robot_state_.dq.size());
    publisher_franka_states_.msg_.dtau_J.resize(robot_state_.dtau_J.size());
    publisher_franka_states_.msg_.EE_F_ext_hat_EE.resize(
        robot_state_.EE_F_ext_hat_EE.size());
    publisher_franka_states_.msg_.elbow_start.resize(
        robot_state_.elbow_start.size());
    publisher_franka_states_.msg_.joint_collision.resize(
        robot_state_.joint_collision.size());
    publisher_franka_states_.msg_.joint_contact.resize(
        robot_state_.joint_contact.size());
    publisher_franka_states_.msg_.O_F_ext_hat_EE.resize(
        robot_state_.O_F_ext_hat_EE.size());
    publisher_franka_states_.msg_.q.resize(robot_state_.q.size());
    publisher_franka_states_.msg_.q_d.resize(robot_state_.q_d.size());
    publisher_franka_states_.msg_.q_start.resize(robot_state_.q_start.size());
    publisher_franka_states_.msg_.tau_ext_hat_filtered.resize(
        robot_state_.tau_ext_hat_filtered.size());
    publisher_franka_states_.msg_.tau_J.resize(robot_state_.tau_J.size());
    publisher_franka_states_.msg_.O_T_EE_start.layout.data_offset = 0;
    publisher_franka_states_.msg_.O_T_EE_start.layout.dim.clear();
    publisher_franka_states_.msg_.O_T_EE_start.layout.dim.push_back(
        std_msgs::MultiArrayDimension());
    publisher_franka_states_.msg_.O_T_EE_start.layout.dim[0].size = 4;
    publisher_franka_states_.msg_.O_T_EE_start.layout.dim[0].stride = 4 * 4;
    publisher_franka_states_.msg_.O_T_EE_start.layout.dim[0].label = "row";
    publisher_franka_states_.msg_.O_T_EE_start.layout.dim.push_back(
        std_msgs::MultiArrayDimension());
    publisher_franka_states_.msg_.O_T_EE_start.layout.dim[1].size = 4;
    publisher_franka_states_.msg_.O_T_EE_start.layout.dim[1].stride = 4;
    publisher_franka_states_.msg_.O_T_EE_start.layout.dim[1].label = "column";
    publisher_franka_states_.msg_.O_T_EE_start.data.resize(16);
    publisher_franka_states_.unlock();
  } else {
    ROS_ERROR("Could not lock publisher_franka_states for resizing.");
  }

  if (publisher_joint_states_.trylock()) {
    publisher_joint_states_.msg_.name.resize(joint_name_.size());
    publisher_joint_states_.msg_.position.resize(robot_state_.q.size());
    publisher_joint_states_.msg_.velocity.resize(robot_state_.dq.size());
    publisher_joint_states_.msg_.effort.resize(robot_state_.tau_J.size());
    publisher_joint_states_.unlock();
  } else {
    ROS_ERROR("Could not lock publisher_joint_states for resizing.");
  }
}

bool franka_hw::FrankaHW::update() {
  try {
    if (robot_.waitForRobotState()) {
      updateStates(robot_.robotState());
      publishFrankaStates();
      publishJointStates();
      return true;
    } else {
      ROS_ERROR_THROTTLE(
          1, "failed to read franka state as connection to robot was closed");
      return false;
    }
  } catch (franka::NetworkException const& e) {
    ROS_ERROR_STREAM("" << e.what());
    return false;
  }
}

/**
* \param robot_state A data struct for franka robot states as received with
* libfranka
*/
void franka_hw::FrankaHW::updateStates(const franka::RobotState& robot_state) {
  robot_state_ = robot_state;
}

void franka_hw::FrankaHW::publishFrankaStates() {
  if (publisher_franka_states_.trylock()) {
    for (size_t i = 0; i < robot_state_.cartesian_collision.size(); ++i) {
      publisher_franka_states_.msg_.cartesian_collision[i] =
          robot_state_.cartesian_collision[i];
      publisher_franka_states_.msg_.cartesian_contact[i] =
          robot_state_.cartesian_contact[i];
      publisher_franka_states_.msg_.EE_F_ext_hat_EE[i] =
          robot_state_.EE_F_ext_hat_EE[i];
      publisher_franka_states_.msg_.O_F_ext_hat_EE[i] =
          robot_state_.O_F_ext_hat_EE[i];
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
      publisher_franka_states_.msg_.q_start[i] = robot_state_.q_start[i];
      publisher_franka_states_.msg_.tau_ext_hat_filtered[i] =
          robot_state_.tau_ext_hat_filtered[i];
    }

    for (size_t i = 0; i < robot_state_.elbow_start.size(); ++i) {
      publisher_franka_states_.msg_.elbow_start[i] =
          robot_state_.elbow_start[i];
    }

    for (size_t row = 0; row < 4; ++row) {
      for (size_t col = 0; col < 4; ++col) {
        publisher_franka_states_.msg_.O_T_EE_start.data[4 * row + col] =
            robot_state_.O_T_EE_start[4 * col + row];
      }
    }

    publisher_franka_states_.msg_.header.seq = sequence_number_franka_states_;
    publisher_franka_states_.msg_.header.stamp = ros::Time::now();
    publisher_franka_states_.unlockAndPublish();
    sequence_number_franka_states_++;
  } else {
    missed_pulishes_franka_states_++;
    ROS_WARN("could not lock franka_states for publishing, missed %i of %i",
             int(missed_pulishes_franka_states_),
             int(sequence_number_franka_states_));
    sequence_number_franka_states_++;  // package loss can be detected when
                                       // seq_nr jumps
  }
}

void franka_hw::FrankaHW::publishJointStates() {
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
    missed_pulishes_joint_states_++;
    ROS_WARN("could not lock joint_states for publishing, missed %i of %i",
             int(missed_pulishes_joint_states_),
             int(sequence_number_joint_states_));
    sequence_number_joint_states_++;  // package loss can be detected when
                                      // seq_nr jumps
  }
}

PLUGINLIB_EXPORT_CLASS(franka_hw::FrankaHW, hardware_interface::RobotHW)
