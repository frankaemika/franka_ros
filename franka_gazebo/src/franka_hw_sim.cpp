#include <franka/duration.h>
#include <franka_gazebo/franka_hw_sim.h>
#include <franka_gazebo/model_kdl.h>
#include <gazebo_ros_control/robot_hw_sim.h>
#include <Eigen/Dense>
#include <iostream>
#include <sstream>
#include <string>

namespace franka_gazebo {

Eigen::Matrix3d skewMatrix(const Eigen::Vector3d& vec) {
  Eigen::Matrix3d vec_hat;
  // clang-format off
  vec_hat <<      0, -vec(2),  vec(1),
             vec(2),    0   , -vec(0),
            -vec(1), vec(0) ,    0   ;
  // clang-format on
  return vec_hat;
}

bool FrankaHWSim::initSim(const std::string& robot_namespace,
                          ros::NodeHandle model_nh,
                          gazebo::physics::ModelPtr parent,
                          const urdf::Model* const urdf,
                          std::vector<transmission_interface::TransmissionInfo> transmissions) {
  this->robot_ = parent;
  gazebo::physics::PhysicsEnginePtr physics = gazebo::physics::get_world()->Physics();
  ROS_INFO_STREAM_NAMED("franka_hw_sim", "Using physics type " << physics->GetType());

  // Generate a list of franka_gazebo::Joint to store all relevant information
  for (auto& transmission : transmissions) {
    if (transmission.type_ != "transmission_interface/SimpleTransmission") {
      continue;
    }
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
    auto joint = std::make_shared<franka_gazebo::Joint>();
    joint->name = transmission.joints_[0].name_;
    if (not urdf) {
      ROS_ERROR_STREAM_NAMED(
          "franka_hw_sim", "Could not find any URDF model. Was it loaded on the parameter server?");
      return false;
    }
    const auto urdfJoint = urdf->getJoint(joint->name);
    if (not urdfJoint) {
      ROS_ERROR_STREAM_NAMED("franka_hw_sim",
                             "Could not get joint '" << joint->name << "' from URDF");
      return false;
    }
    joint->type = urdfJoint->type;
    joint->axis = Eigen::Vector3d(urdfJoint->axis.x, urdfJoint->axis.y, urdfJoint->axis.z);

    // Get a handle to the underlying Gazebo Joint
    gazebo::physics::JointPtr handle = parent->GetJoint(joint->name);
    if (not handle) {
      ROS_ERROR_STREAM_NAMED("franka_hw_sim", "This robot has a joint named '"
                                                  << joint->name
                                                  << "' which is not in the gazebo model.");
      return false;
    }
    joint->handle = handle;
    this->joints_.emplace(joint->name, joint);
  }

  // After the joint data containers have been fully initialized and their memory address don't
  // change anymore, get the respective addresses to pass them to the handles
  for (auto& pair : this->joints_) {
    auto joint = pair.second;

    // Register the state interface
    this->jsi_.registerHandle(hardware_interface::JointStateHandle(
        joint->name, &joint->position, &joint->velocity, &joint->effort));
  }

  // Register all supported command interfaces
  for (auto& transmission : transmissions) {
    for (const auto interface : transmission.joints_[0].hardware_interfaces_) {
      auto joint = this->joints_[transmission.joints_[0].name_];
      if (transmission.type_ == "transmission_interface/SimpleTransmission") {
        ROS_INFO_STREAM_NAMED("franka_hw_sim", "Found transmission interface of joint '"
                                                   << joint->name << "': " << interface);
        if (interface == "hardware_interface/EffortJointInterface") {
          this->eji_.registerHandle(
              hardware_interface::JointHandle(this->jsi_.getHandle(joint->name), &joint->command));
          continue;
        }
      }

      if (transmission.type_ == "franka_hw/FrankaStateInterface") {
        ROS_INFO_STREAM_NAMED("franka_hw_sim",
                              "Found transmission interface '" << transmission.type_ << "'");
        if (transmission.joints_.size() != 7) {
          ROS_ERROR_STREAM_NAMED(
              "franka_hw_sim",
              "Cannot create franka_hw/FrankaStateInterface for robot '"
                  << robot_namespace << "_robot' because " << transmission.joints_.size()
                  << " joints were found beneath the <transmission> tag, but 7 are required.");
          return false;
        }

        // Check if all joints defined in the <transmission> actually exist in the URDF
        for (auto& joint : transmission.joints_) {
          if (this->joints_.count(joint.name_) == 0) {
            ROS_ERROR_STREAM_NAMED(
                "franka_hw_sim", "Cannot create franka_hw/FrankaStateInterface for robot '"
                                     << robot_namespace + "_robot' because the specified joint '"
                                     << joint.name_
                                     << "' in the <transmission> tag cannot be found in the URDF");
            return false;
          }
          ROS_INFO_STREAM_NAMED("franka_hw_sim",
                                "Found joint " << joint.name_ << " to belong to a Panda robot");
          this->names_.push_back(joint.name_);
        }
        this->fsi_.registerHandle(
            franka_hw::FrankaStateHandle(robot_namespace + "_robot", this->robot_state_));
        continue;
      }

      if (transmission.type_ == "franka_hw/FrankaModelInterface") {
        ROS_INFO_STREAM_NAMED("franka_hw_sim",
                              "Found transmission interface '" << transmission.type_ << "'");
        if (transmission.joints_.size() != 2) {
          ROS_ERROR_STREAM_NAMED(
              "franka_hw_sim",
              "Cannot create franka_hw/FrankaModelInterface for robot '"
                  << robot_namespace << "_model' because " << transmission.joints_.size()
                  << " joints were found beneath the <transmission> tag, but 2 are required.");
          return false;
        }

        for (auto& joint : transmission.joints_) {
          if (this->joints_.count(joint.name_) == 0) {
            ROS_ERROR_STREAM_NAMED(
                "franka_hw_sim", "Cannot create franka_hw/FrankaModelInterface for robot '"
                                     << robot_namespace << "_model' because the specified joint '"
                                     << joint.name_
                                     << "' in the <transmission> tag cannot be found in the URDF");
            return false;
          }
        }
        auto root = std::find_if(
            transmission.joints_.begin(), transmission.joints_.end(),
            [&](const transmission_interface::JointInfo& i) { return i.role_ == "root"; });
        if (root == transmission.joints_.end()) {
          ROS_ERROR_STREAM_NAMED(
              "franka_hw_sim",
              "Cannot create franka_hw/FrankaModelInterface for robot '"
                  << robot_namespace
                  << "_model' because no <joint> with <role>root</root> can be found "
                     "in the <transmission>");
          return false;
        }
        auto tip = std::find_if(
            transmission.joints_.begin(), transmission.joints_.end(),
            [&](const transmission_interface::JointInfo& i) { return i.role_ == "tip"; });
        if (tip == transmission.joints_.end()) {
          ROS_ERROR_STREAM_NAMED(
              "franka_hw_sim",
              "Cannot create franka_hw/FrankaModelInterface for robot '"
                  << robot_namespace
                  << "_model' because no <joint> with <role>tip</role> can be found "
                     "in the <transmission>");
          return false;
        }
        try {
          auto rootLink = urdf->getJoint(root->name_)->parent_link_name;
          auto tipLink = urdf->getJoint(tip->name_)->child_link_name;

          this->model_ = std::make_unique<franka_gazebo::ModelKDL>(*urdf, rootLink, tipLink);

          this->fmi_.registerHandle(franka_hw::FrankaModelHandle(
              robot_namespace + "_model", *this->model_, this->robot_state_));
        } catch (const std::invalid_argument& e) {
          ROS_ERROR_STREAM_NAMED("franka_hw_sim",
                                 "Cannot create franka_hw/FrankaModelInterface for robot '"
                                     << robot_namespace << "_model'. " << e.what());
          return false;
        }
        continue;
      }
      ROS_WARN_STREAM_NAMED("franka_hw_sim", "Unsupported transmission interface of joint '"
                                                 << joint->name << "': " << interface);
    }
  }

  registerInterface(&this->jsi_);
  registerInterface(&this->eji_);
  registerInterface(&this->fsi_);
  registerInterface(&this->fmi_);

  if (not readParameters(model_nh)) {
    return false;
  }

  return true;
}

void FrankaHWSim::readSim(ros::Time time, ros::Duration period) {
  for (auto& pair : this->joints_) {
    auto joint = pair.second;
    joint->update(period);
  }
  this->updateRobotState(time);
}

void FrankaHWSim::writeSim(ros::Time time, ros::Duration period) {
  auto g = this->model_->gravity(this->robot_state_);

  for (auto& pair : this->joints_) {
    auto joint = pair.second;
    auto command = joint->command;

    // Check if this joint is affected by gravity compensation. That is the case, if it is part
    // of the list of joints of the FrankaStateInterface
    auto index = std::find(this->names_.begin(), this->names_.end(), pair.first);
    if (index != this->names_.end()) {
      int i = index - this->names_.begin();
      command += g.at(i);
    }

    if (std::isnan(command)) {
      ROS_WARN_STREAM_NAMED("franka_hw_sim",
                            "Command for " << joint->name << "is NaN, won't send to robot");
      continue;
    }
    joint->handle->SetForce(0, command);
  }
}

void FrankaHWSim::eStopActive(bool active) {}

bool FrankaHWSim::readParameters(ros::NodeHandle nh) {
  nh.param<double>("m_ee", this->robot_state_.m_ee, 0.73);

  try {
    std::string I_ee;
    nh.param<std::string>("I_ee", I_ee, "0.001 0 0 0 0.0025 0 0 0 0.0017");
    this->robot_state_.I_ee = readArray<9>(I_ee, "I_ee");
    nh.param<double>("m_load", this->robot_state_.m_load, 0);

    std::string I_load;
    nh.param<std::string>("I_load", I_load, "0 0 0 0 0 0 0 0 0");
    this->robot_state_.I_load = readArray<9>(I_load, "I_load");

    std::string F_x_Cload;
    nh.param<std::string>("F_x_Cload", F_x_Cload, "0 0 0");
    this->robot_state_.F_x_Cload = readArray<3>(F_x_Cload, "F_x_Cload");

    std::string F_T_EE;
    nh.param<std::string>("F_T_EE", F_T_EE,
                          "0.7071 -0.7071 0 0 0.7071 0.7071 0 0 0 0 1 0 0 0 0.1034 1");
    this->robot_state_.F_T_EE = readArray<16>(F_T_EE, "F_T_EE");

    std::string conts, colts;
    nh.param<std::string>("contact_torque_thresholds", conts, "INF INF INF INF INF INF INF");
    nh.param<std::string>("collision_torque_thresholds", colts, "INF INF INF INF INF INF INF");
    auto contact_thresholds = readArray<7>(conts, "contact_torque_thresholds");
    auto collision_thresholds = readArray<7>(colts, "collision_torque_thresholds");
    // TODO: Lookup the thresholds by name not index
    for (int i = 0; i < 7; i++) {
      std::string name = this->names_.at(i);
      this->joints_[name]->contact_threshold = contact_thresholds.at(i);
      this->joints_[name]->collision_threshold = collision_thresholds.at(i);
    }

  } catch (const std::invalid_argument& e) {
    ROS_ERROR_STREAM_NAMED("franka_hw_sim", e.what());
    return false;
  }
  this->robot_state_.m_total = this->robot_state_.m_ee + this->robot_state_.m_load;

  this->robot_state_.NE_T_EE = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
  this->robot_state_.F_T_NE = this->robot_state_.F_T_EE;
  this->robot_state_.EE_T_K = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};

  // Compute I_total by convert I_ee into load frame using Theorem of Steiner:
  // https://de.wikipedia.org/wiki/Steinerscher_Satz#Verallgemeinerung_auf_Tr%C3%A4gheitstensoren
  Eigen::Vector3d a(this->robot_state_.F_x_Cload.data());
  Eigen::Matrix3d a_ = skewMatrix(a);
  Eigen::Matrix3d Is(this->robot_state_.I_ee.data());
  Eigen::Matrix3d I = Is + this->robot_state_.m_ee * a_.transpose() * a_;
  Eigen::Map<Eigen::Matrix3d>(this->robot_state_.I_total.data()) = I;

  return true;
}

void FrankaHWSim::updateRobotState(ros::Time time) {
  // This is ensured, because a FrankaStateInterface checks exactly for seven joints in the URDF
  assert(this->names_.size() == 7);
  assert(this->joints_.size() == 7);

  for (int i = 0; i < 7; i++) {
    std::string name = this->names_.at(i);
    const auto& joint = this->joints_.at(name);
    this->robot_state_.q[i] = joint->position;
    this->robot_state_.dq[i] = joint->velocity;
    this->robot_state_.tau_J[i] = joint->effort;
    this->robot_state_.dtau_J[i] = joint->jerk;

    this->robot_state_.q_d[i] = joint->position;
    this->robot_state_.dq_d[i] = joint->velocity;
    this->robot_state_.ddq_d[i] = joint->acceleration;
    this->robot_state_.tau_J_d[i] = joint->effort;

    // For now we assume no flexible joints
    this->robot_state_.theta[i] = joint->position;
    this->robot_state_.dtheta[i] = joint->velocity;

    // TODO: Add configurable noise here?
    // TODO: Add filter
    this->robot_state_.tau_ext_hat_filtered[i] = joint->effort - joint->command;

    this->robot_state_.joint_contact[i] = joint->isInContact();
    this->robot_state_.joint_collision[i] = joint->isInCollision();
  }

  this->robot_state_.control_command_success_rate = 1.0;
  this->robot_state_.time = franka::Duration(time.toNSec() / 1e6 /*ms*/);

  auto world_T_robot = this->robot_->WorldPose();
  auto world_T_ee = this->joints_.at(this->names_.at(6))->handle->GetChild()->WorldCoGPose();
  auto robot_T_ee = world_T_robot - world_T_ee;
  auto t = robot_T_ee.Pos();
  auto q = robot_T_ee.Rot();

  Eigen::Affine3d T_ee;
  T_ee.fromPositionOrientationScale(Eigen::Vector3d(t.X(), t.Y(), t.Z()),
                                    Eigen::Quaterniond(q.W(), q.X(), q.Y(), q.Z()),
                                    Eigen::Vector3d(1, 1, 1));
  // clang-format off
  // Column-Major format for libfranka's Robot State
  this->robot_state_.O_T_EE = {
    T_ee(0,0), T_ee(1,0), T_ee(2,0), T_ee(3,0),
    T_ee(0,1), T_ee(1,1), T_ee(2,1), T_ee(3,1),
    T_ee(0,2), T_ee(1,2), T_ee(2,2), T_ee(3,2),
    T_ee(0,3), T_ee(1,3), T_ee(2,3), T_ee(3,3)
  };
  // clang-format on
}

}  // namespace franka_gazebo

PLUGINLIB_EXPORT_CLASS(franka_gazebo::FrankaHWSim, gazebo_ros_control::RobotHWSim)
