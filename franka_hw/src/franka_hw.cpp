#include <franka_hw/franka_hw.h>

#include <functional>
#include <mutex>

#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <realtime_tools/realtime_publisher.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/MultiArrayDimension.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <urdf/model.h>

#include "franka_hw_helper_functions.h"

namespace franka_hw {

constexpr double FrankaHW::kMaximumJointAcceleration;
constexpr double FrankaHW::kMaximumJointJerk;

FrankaHW::FrankaHW(const std::vector<std::string>& joint_names,
                   franka::Robot* robot,
                   const std::string& arm_id,
                   const ros::NodeHandle& node_handle)
    : joint_state_interface_(),
      franka_joint_state_interface_(),
      franka_cartesian_state_interface_(),
      position_joint_interface_(),
      velocity_joint_interface_(),
      effort_joint_interface_(),
      franka_pose_cartesian_interface_(),
      franka_velocity_cartesian_interface_(),
      franka_model_interface_(),
      position_joint_limit_interface_(),
      velocity_joint_limit_interface_(),
      effort_joint_limit_interface_(),
      publisher_transforms_(node_handle, "/tf", 1),
      publisher_franka_states_(node_handle, "franka_states", 1),
      publisher_joint_states_(node_handle, "joint_states", 1),
      publisher_external_wrench_(node_handle, "F_ext", 1),
      joint_names_(),
      arm_id_(arm_id),
      robot_(robot),
      model_(nullptr),
      robot_state_(),
      position_joint_command_({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}),
      velocity_joint_command_({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}),
      effort_joint_command_({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}),
      pose_cartesian_command_({1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
                               1.0, 0.0, 0.0, 0.0, 0.0, 1.0}),
      velocity_cartesian_command_({0.0, 0.0, 0.0, 0.0, 0.0, 0.0}),
      default_run_function_([this](std::function<bool()> ros_callback) {
        robot_->read(std::bind(&FrankaHW::readCallback, this, ros_callback,
                               std::placeholders::_1));
      }),
      run_function_(default_run_function_) {
  joint_names_.resize(joint_names.size());
  joint_names_ = joint_names;

  for (size_t i = 0; i < joint_names_.size(); ++i) {
    hardware_interface::JointStateHandle joint_handle(
        joint_names_[i], &robot_state_.q_d[i], &robot_state_.dq[i],
        &robot_state_.tau_J[i]);
    joint_state_interface_.registerHandle(joint_handle);

    FrankaJointStateHandle franka_joint_handle(
        joint_names_[i], robot_state_.q[i], robot_state_.dq[i],
        robot_state_.tau_J[i], robot_state_.q_d[i], robot_state_.dtau_J[i],
        robot_state_.tau_ext_hat_filtered[i], robot_state_.joint_collision[i],
        robot_state_.joint_contact[i]);
    franka_joint_state_interface_.registerHandle(franka_joint_handle);

    hardware_interface::JointHandle position_joint_handle(
        joint_state_interface_.getHandle(joint_names_[i]),
        &position_joint_command_.q[i]);
    position_joint_interface_.registerHandle(position_joint_handle);

    hardware_interface::JointHandle velocity_joint_handle(
        joint_state_interface_.getHandle(joint_names_[i]),
        &velocity_joint_command_.dq[i]);
    velocity_joint_interface_.registerHandle(velocity_joint_handle);

    hardware_interface::JointHandle effort_joint_handle(
        joint_state_interface_.getHandle(joint_names_[i]),
        &effort_joint_command_.tau_J[i]);
    effort_joint_interface_.registerHandle(effort_joint_handle);
  }

  if (node_handle.hasParam("robot_description")) {
    urdf::Model urdf_model;
    if (!urdf_model.initParamWithNodeHandle("robot_description", node_handle)) {
      ROS_ERROR("Could not initialize urdf model from robot_description");
    } else {
      joint_limits_interface::SoftJointLimits soft_limits;
      joint_limits_interface::JointLimits joint_limits;

      for (auto joint_name : joint_names_) {
        boost::shared_ptr<const urdf::Joint> urdf_joint =
            urdf_model.getJoint(joint_name);
        if (!urdf_joint) {
          ROS_ERROR_STREAM("Could not get joint " << joint_name
                                                  << " from urdf");
        }
        if (!urdf_joint->safety) {
          ROS_ERROR_STREAM("Joint " << joint_name << " has no safety");
        }
        if (!urdf_joint->limits) {
          ROS_ERROR_STREAM("Joint " << joint_name << " has no limits");
        }

        if (joint_limits_interface::getSoftJointLimits(urdf_joint,
                                                       soft_limits)) {
          if (joint_limits_interface::getJointLimits(urdf_joint,
                                                     joint_limits)) {
            joint_limits.max_acceleration = kMaximumJointAcceleration;
            joint_limits.has_acceleration_limits = true;
            joint_limits.max_jerk = kMaximumJointJerk;
            joint_limits.has_jerk_limits = true;
            joint_limits_interface::PositionJointSoftLimitsHandle
                position_limit_handle(
                    position_joint_interface_.getHandle(joint_name),
                    joint_limits, soft_limits);
            position_joint_limit_interface_.registerHandle(
                position_limit_handle);

            joint_limits_interface::VelocityJointSoftLimitsHandle
                velocity_limit_handle(
                    velocity_joint_interface_.getHandle(joint_name),
                    joint_limits, soft_limits);
            velocity_joint_limit_interface_.registerHandle(
                velocity_limit_handle);

            joint_limits_interface::EffortJointSoftLimitsHandle
                effort_limit_handle(
                    effort_joint_interface_.getHandle(joint_name), joint_limits,
                    soft_limits);
            effort_joint_limit_interface_.registerHandle(effort_limit_handle);
          } else {
            ROS_ERROR_STREAM("Could not parse joint limit for joint "
                             << joint_name << " for joint limit interfaces");
          }
        } else {
          ROS_ERROR_STREAM("Could not parse soft joint limit for joint "
                           << joint_name << " for joint limit interfaces");
        }
      }
    }
  } else {
    ROS_WARN("No parameter robot_description found to set joint limits!");
  }
  FrankaCartesianStateHandle franka_cartesian_state_handle(
      arm_id_ + "_cartesian", robot_state_.cartesian_collision,
      robot_state_.cartesian_contact, robot_state_.O_F_ext_hat_K,
      robot_state_.K_F_ext_hat_K, robot_state_.O_T_EE, robot_state_.O_T_EE_d);
  franka_cartesian_state_interface_.registerHandle(
      franka_cartesian_state_handle);
  FrankaCartesianPoseHandle franka_cartesian_pose_handle(
      franka_cartesian_state_interface_.getHandle(arm_id_ + "_cartesian"),
      pose_cartesian_command_.O_T_EE);
  franka_pose_cartesian_interface_.registerHandle(franka_cartesian_pose_handle);
  FrankaCartesianVelocityHandle franka_cartesian_velocity_handle(
      franka_cartesian_state_interface_.getHandle(arm_id_ + "_cartesian"),
      velocity_cartesian_command_.O_dP_EE);
  franka_velocity_cartesian_interface_.registerHandle(
      franka_cartesian_velocity_handle);

  franka_hw::FrankaModelHandle model_handle(arm_id_ + "model", &model_,
                                            robot_state_);
  franka_model_interface_.registerHandle(model_handle);

  registerInterface(&franka_model_interface_);
  registerInterface(&joint_state_interface_);
  registerInterface(&franka_joint_state_interface_);
  registerInterface(&franka_cartesian_state_interface_);
  registerInterface(&position_joint_interface_);
  registerInterface(&velocity_joint_interface_);
  registerInterface(&effort_joint_interface_);
  registerInterface(&franka_pose_cartesian_interface_);
  registerInterface(&franka_velocity_cartesian_interface_);

  {
    std::lock_guard<realtime_tools::RealtimePublisher<FrankaState> > lock(
        publisher_franka_states_);
    publisher_franka_states_.msg_.cartesian_collision.resize(
        robot_state_.cartesian_collision.size());
    publisher_franka_states_.msg_.cartesian_contact.resize(
        robot_state_.cartesian_contact.size());
    publisher_franka_states_.msg_.dq.resize(robot_state_.dq.size());
    publisher_franka_states_.msg_.dtau_J.resize(robot_state_.dtau_J.size());
    publisher_franka_states_.msg_.K_F_ext_hat_K.resize(
        robot_state_.K_F_ext_hat_K.size());
    publisher_franka_states_.msg_.elbow.resize(robot_state_.elbow.size());
    publisher_franka_states_.msg_.elbow_d.resize(robot_state_.elbow_d.size());
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
    publisher_franka_states_.msg_.O_T_EE_d.layout.data_offset = 0;
    publisher_franka_states_.msg_.O_T_EE_d.layout.dim.clear();
    publisher_franka_states_.msg_.O_T_EE_d.layout.dim.push_back(
        std_msgs::MultiArrayDimension());
    publisher_franka_states_.msg_.O_T_EE_d.layout.dim[0].size = 4;
    publisher_franka_states_.msg_.O_T_EE_d.layout.dim[0].stride = 4 * 4;
    publisher_franka_states_.msg_.O_T_EE_d.layout.dim[0].label = "row";
    publisher_franka_states_.msg_.O_T_EE_d.layout.dim.push_back(
        std_msgs::MultiArrayDimension());
    publisher_franka_states_.msg_.O_T_EE_d.layout.dim[1].size = 4;
    publisher_franka_states_.msg_.O_T_EE_d.layout.dim[1].stride = 4;
    publisher_franka_states_.msg_.O_T_EE_d.layout.dim[1].label = "column";
    publisher_franka_states_.msg_.O_T_EE_d.data.resize(16);
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

void FrankaHW::run(std::function<bool()> ros_callback) {
  if (robot_ == nullptr) {
    throw std::invalid_argument("franka::Robot was not initialized.");
  }

  if (model_ == nullptr) {
    try {
      model_.reset(new franka::Model(robot_->loadModel()));
    } catch (franka::ModelException& e) {
      ROS_ERROR_STREAM(
          "FrankaHW: Exception loading model from robot: " << e.what());
    }
  }

  uint32_t last_sequence_number = 0;
  do {
    run_function_([this, ros_callback, &last_sequence_number]() {
      if (last_sequence_number != robot_state_.sequence_number) {
        last_sequence_number = robot_state_.sequence_number;
        return ros_callback();
      }
      return true;
    });
  } while (ros_callback());
  // TODO(FWA): how to handle e.g. collisions in ros_control?
  // Currently just propagates exceptions.
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

    for (size_t i = 0; i < robot_state_.elbow_d.size(); ++i) {
      publisher_franka_states_.msg_.elbow_d[i] = robot_state_.elbow_d[i];
    }

    for (size_t row = 0; row < 4; ++row) {
      for (size_t col = 0; col < 4; ++col) {
        publisher_franka_states_.msg_.O_T_EE.data[4 * row + col] =
            robot_state_.O_T_EE[4 * col + row];
      }
    }

    for (size_t row = 0; row < 4; ++row) {
      for (size_t col = 0; col < 4; ++col) {
        publisher_franka_states_.msg_.O_T_EE_d.data[4 * row + col] =
            robot_state_.O_T_EE_d[4 * col + row];
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
      publisher_joint_states_.msg_.position[i] = robot_state_.q_d[i];
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

void FrankaHW::enforceLimits(const ros::Duration kPeriod) {
  position_joint_limit_interface_.enforceLimits(kPeriod);
  velocity_joint_limit_interface_.enforceLimits(kPeriod);
  effort_joint_limit_interface_.enforceLimits(kPeriod);
}

bool FrankaHW::checkForConflict(
    const std::list<hardware_interface::ControllerInfo>& info) const {
  ResourceWithClaimsMap resource_map = getResourceMap(info);
  // check for conflicts in single resources: no triple claims,
  // for 2 claims it must be one torque and one non-torque claim
  for (auto map_it = resource_map.begin(); map_it != resource_map.end();
       map_it++) {
    if (map_it->second.size() > 2) {
      ROS_ERROR_STREAM("Resource "
                       << map_it->first
                       << " claimed with more than two interfaces. Conflict!");
      return true;
    }
    u_int8_t torque_claims = 0;
    u_int8_t other_claims = 0;
    if (map_it->second.size() == 2) {
      for (auto& claimed_by : map_it->second) {
        if (claimed_by[2].compare("hardware_interface::EffortJointInterface") ==
            0) {
          torque_claims++;
        } else {
          other_claims++;
        }
      }
      if (torque_claims != 1) {
        ROS_ERROR_STREAM(
            "Resource "
            << map_it->first
            << " is claimed with two non-compatible interfaces. Conflict!");
        return true;
      }
    }
  }

  ArmClaimedMap arm_claim_map;
  if (!getArmClaimedMap(resource_map, arm_claim_map)) {
    ROS_ERROR_STREAM("Unknown interface claimed. Conflict!");
    return true;
  }

  // check for conflicts between joint and cartesian level for each arm.
  // Valid claims are torque claims on joint level in combination with either
  // 7 non-torque claims on joint_level or one claim on cartesian level.
  if (arm_claim_map.find(arm_id_) != arm_claim_map.end()) {
    if ((arm_claim_map[arm_id_].cartesian_velocity_claims +
                 arm_claim_map[arm_id_].cartesian_pose_claims >
             0 &&
         arm_claim_map[arm_id_].joint_position_claims +
                 arm_claim_map[arm_id_].joint_velocity_claims >
             0)) {
      ROS_ERROR_STREAM("Invalid claims on joint AND cartesian level on arm "
                       << arm_id_ << ". Conflict!");
      return true;
    }
    if ((arm_claim_map[arm_id_].joint_position_claims > 0 &&
         arm_claim_map[arm_id_].joint_position_claims != 7) ||
        (arm_claim_map[arm_id_].joint_velocity_claims > 0 &&
         arm_claim_map[arm_id_].joint_velocity_claims != 7) ||
        (arm_claim_map[arm_id_].joint_torque_claims > 0 &&
         arm_claim_map[arm_id_].joint_torque_claims != 7)) {
      ROS_ERROR_STREAM("Non-consistent claims on the joints of "
                       << arm_id_ << ". Not supported. Conflict!");
      return true;
    }
  }
  return false;
}

void FrankaHW::doSwitch(
    const std::list<hardware_interface::ControllerInfo>& start_list,   // NOLINT
    const std::list<hardware_interface::ControllerInfo>& stop_list) {  // NOLINT
  position_joint_limit_interface_.reset();
  controller_running_ = true;
}

bool FrankaHW::prepareSwitch(
    const std::list<hardware_interface::ControllerInfo>& start_list,
    const std::list<hardware_interface::ControllerInfo>& stop_list) {  // NOLINT
  ResourceWithClaimsMap start_resource_map = getResourceMap(start_list);
  ArmClaimedMap start_arm_claim_map;
  if (!getArmClaimedMap(start_resource_map, start_arm_claim_map)) {
    ROS_ERROR("Unknown interface claimed for starting!");
    return false;
  }
  ControlMode start_control_mode = getControlMode(arm_id_, start_arm_claim_map);

  ResourceWithClaimsMap stop_resource_map = getResourceMap(stop_list);
  ArmClaimedMap stop_arm_claim_map;
  if (!getArmClaimedMap(stop_resource_map, stop_arm_claim_map)) {
    ROS_ERROR("Unknown interface claimed for stopping!");
    return false;
  }
  ControlMode stop_control_mode = getControlMode(arm_id_, stop_arm_claim_map);

  ControlMode requested_control_mode = current_control_mode_;
  requested_control_mode |= start_control_mode;
  requested_control_mode &= ~stop_control_mode;

  switch (requested_control_mode) {
    case ControlMode::None:
      run_function_ = default_run_function_;
      break;
    case ControlMode::JointTorque:
      run_function_ = [this](std::function<bool()> ros_callback) {
        robot_->control(std::bind(&FrankaHW::controlCallback<franka::Torques>,
                                  this, std::cref(effort_joint_command_),
                                  ros_callback, std::placeholders::_1));
      };
      break;
    case ControlMode::JointPosition:
      run_function_ = [this](std::function<bool()> ros_callback) {
        robot_->control(
            std::bind(&FrankaHW::controlCallback<franka::JointPositions>, this,
                      std::cref(position_joint_command_), ros_callback,
                      std::placeholders::_1));
      };
      break;
    case ControlMode::JointVelocity:
      run_function_ = [this](std::function<bool()> ros_callback) {
        robot_->control(
            std::bind(&FrankaHW::controlCallback<franka::JointVelocities>, this,
                      std::cref(velocity_joint_command_), ros_callback,
                      std::placeholders::_1));
      };
      break;
    case ControlMode::CartesianPose:
      run_function_ = [this](std::function<bool()> ros_callback) {
        robot_->control(
            std::bind(&FrankaHW::controlCallback<franka::CartesianPose>, this,
                      std::cref(pose_cartesian_command_), ros_callback,
                      std::placeholders::_1));
      };
      break;
    case ControlMode::CartesianVelocity:
      run_function_ = [this](std::function<bool()> ros_callback) {
        robot_->control(
            std::bind(&FrankaHW::controlCallback<franka::CartesianVelocities>,
                      this, std::cref(velocity_cartesian_command_),
                      ros_callback, std::placeholders::_1));
      };
      break;
    case (ControlMode::JointTorque | ControlMode::JointPosition):
      run_function_ = [this](std::function<bool()> ros_callback) {
        robot_->control(
            std::bind(&FrankaHW::controlCallback<franka::JointPositions>, this,
                      std::cref(position_joint_command_), ros_callback,
                      std::placeholders::_1),
            std::bind(&FrankaHW::controlCallback<franka::Torques>, this,
                      std::cref(effort_joint_command_), ros_callback,
                      std::placeholders::_1));
      };
      break;
    case (ControlMode::JointTorque | ControlMode::JointVelocity):
      run_function_ = [this](std::function<bool()> ros_callback) {
        robot_->control(
            std::bind(&FrankaHW::controlCallback<franka::JointVelocities>, this,
                      std::cref(velocity_joint_command_), ros_callback,
                      std::placeholders::_1),
            std::bind(&FrankaHW::controlCallback<franka::Torques>, this,
                      std::cref(effort_joint_command_), ros_callback,
                      std::placeholders::_1));
      };
      break;
    case (ControlMode::JointTorque | ControlMode::CartesianPose):
      run_function_ = [this](std::function<bool()> ros_callback) {
        robot_->control(
            std::bind(&FrankaHW::controlCallback<franka::CartesianPose>, this,
                      std::cref(pose_cartesian_command_), ros_callback,
                      std::placeholders::_1),
            std::bind(&FrankaHW::controlCallback<franka::Torques>, this,
                      std::cref(effort_joint_command_), ros_callback,
                      std::placeholders::_1));
      };
      break;
    case (ControlMode::JointTorque | ControlMode::CartesianVelocity):
      run_function_ = [this](std::function<bool()> ros_callback) {
        robot_->control(
            std::bind(&FrankaHW::controlCallback<franka::CartesianVelocities>,
                      this, std::cref(velocity_cartesian_command_),
                      ros_callback, std::placeholders::_1),
            std::bind(&FrankaHW::controlCallback<franka::Torques>, this,
                      std::cref(effort_joint_command_), ros_callback,
                      std::placeholders::_1));
      };
      break;
    default:
      ROS_WARN("No valid control mode selected; cannot switch controllers.");
      return false;
  }

  ROS_INFO_STREAM("Prepared switching controllers to "
                  << requested_control_mode);
  current_control_mode_ = requested_control_mode;
  controller_running_ = false;
  return true;
}

bool FrankaHW::readCallback(std::function<bool()> ros_callback,
                            const franka::RobotState& robot_state) {
  robot_state_ = robot_state;
  if (!controller_running_) {
    // A new controller has been prepared; stop this one.
    return false;
  }
  if (ros_callback && !ros_callback()) {
    return false;
  }
  return true;
}

std::array<double, 7> FrankaHW::getJointPositionCommand() const {
  return position_joint_command_.q;
}

std::array<double, 7> FrankaHW::getJointVelocityCommand() const {
  return velocity_joint_command_.dq;
}

std::array<double, 7> FrankaHW::getJointEffortCommand() const {
  return effort_joint_command_.tau_J;
}

}  // namespace franka_hw
