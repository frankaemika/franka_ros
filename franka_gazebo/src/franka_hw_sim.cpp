#include <franka_gazebo/franka_hw_sim.h>

#include <franka/duration.h>
#include <franka_example_controllers/pseudo_inversion.h>
#include <franka_gazebo/model_kdl.h>
#include <franka_hw/franka_hw.h>
#include <franka_hw/services.h>
#include <franka_msgs/SetEEFrame.h>
#include <franka_msgs/SetForceTorqueCollisionBehavior.h>
#include <franka_msgs/SetJointConfiguration.h>
#include <franka_msgs/SetKFrame.h>
#include <franka_msgs/SetLoad.h>
#include <gazebo_msgs/SetModelConfiguration.h>
#include <gazebo_ros_control/robot_hw_sim.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <Eigen/Dense>
#include <boost/algorithm/clamp.hpp>
#include <iostream>
#include <sstream>
#include <string>
#include <boost/algorithm/clamp.hpp>
#include "ros/ros.h"

namespace franka_gazebo {

bool FrankaHWSim::initSim(const std::string& robot_namespace,
                          ros::NodeHandle model_nh,
                          gazebo::physics::ModelPtr parent,
                          const urdf::Model* const urdf,
                          std::vector<transmission_interface::TransmissionInfo> transmissions) {
  model_nh.param<std::string>("arm_id", this->arm_id_, robot_namespace);
  if (this->arm_id_ != robot_namespace) {
    ROS_WARN_STREAM_NAMED(
        "franka_hw_sim",
        "Caution: Robot names differ! Read 'arm_id: "
            << this->arm_id_ << "' from parameter server but URDF defines '<robotNamespace>"
            << robot_namespace << "</robotNamespace>'. Will use '" << this->arm_id_ << "'!");
  }

  this->robot_ = parent;
  this->efforts_initialized_ = false;

#if GAZEBO_MAJOR_VERSION >= 8
  gazebo::physics::PhysicsEnginePtr physics = gazebo::physics::get_world()->Physics();
#else
  gazebo::physics::PhysicsEnginePtr physics = gazebo::physics::get_world()->GetPhysicsEngine();
#endif

  // Print information about the used physics engine
  std::vector<std::string> supported_engines{"ode", "dart"};
  std::string physics_engine = physics->GetType();
  ROS_INFO_STREAM_NAMED("franka_hw_sim", "Using physics type " << physics->GetType());
  if (std::find(supported_engines.begin(), supported_engines.end(), physics_engine) ==
      supported_engines.end()) {
    ROS_ERROR_STREAM_NAMED("franka_hw_sim",
                           "The Panda Gazebo model does not yet officially support the '" +
                               physics_engine + "' physics engine.");
  }

  // Retrieve initial gravity vector from Gazebo
  // NOTE: Can be overwritten by the user via the 'gravity_vector' ROS parameter.
  auto gravity = physics->World()->Gravity();
  this->gravity_earth_ = {gravity.X(), gravity.Y(), gravity.Z()};

  model_nh.param<double>("tau_ext_lowpass_filter", this->tau_ext_lowpass_filter_,
                         kDefaultTauExtLowpassFilter);

  // Generate a list of franka_gazebo::Joint to store all relevant information
  for (const auto& transmission : transmissions) {
    if (transmission.type_ != "transmission_interface/SimpleTransmission") {
      continue;
    }
    if (transmission.joints_.empty()) {
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
    if (urdf == nullptr) {
      ROS_ERROR_STREAM_NAMED(
          "franka_hw_sim", "Could not find any URDF model. Was it loaded on the parameter server?");
      return false;
    }
    auto urdf_joint = urdf->getJoint(joint->name);
    if (not urdf_joint) {
      ROS_ERROR_STREAM_NAMED("franka_hw_sim",
                             "Could not get joint '" << joint->name << "' from URDF");
      return false;
    }
    joint->type = urdf_joint->type;
    joint_limits_interface::getJointLimits(urdf_joint, joint->limits);
    joint->axis = Eigen::Vector3d(urdf_joint->axis.x, urdf_joint->axis.y, urdf_joint->axis.z);

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
    initJointStateHandle(pair.second);
  }

  // Register all supported command interfaces
  for (auto& transmission : transmissions) {
    for (const auto& k_interface : transmission.joints_[0].hardware_interfaces_) {
      auto joint = this->joints_[transmission.joints_[0].name_];
      if (transmission.type_ == "transmission_interface/SimpleTransmission") {
        ROS_INFO_STREAM_NAMED("franka_hw_sim", "Found transmission interface of joint '"
                                                   << joint->name << "': " << k_interface);
        if (k_interface == "hardware_interface/EffortJointInterface") {
          this->joint_control_methods_.emplace(joint->name, EFFORT);

          initEffortCommandHandle(joint);
          continue;
        }
        if (k_interface == "hardware_interface/PositionJointInterface") {
          this->joint_control_methods_.emplace(joint->name, POSITION);

          // Initiate position motion generator (PID controller)
          const ros::NodeHandle kPosPidGainsNh(robot_namespace +
                                               "/motion_generators/position/gains/" + joint->name);
          control_toolbox::Pid pid;
          this->pid_controllers_.emplace(joint->name, pid);
          this->pid_controllers_[joint->name].init(kPosPidGainsNh);

          initPositionCommandHandle(joint);
          continue;
        }
        if (k_interface == "hardware_interface/VelocityJointInterface") {
          this->joint_control_methods_.emplace(joint->name, VELOCITY);

          // Initiate velocity motion generator (PID controller)
          const ros::NodeHandle kVelPidGainsNh(robot_namespace +
                                               "/motion_generators/velocity/gains/" + joint->name);
          control_toolbox::Pid pid;
          this->pid_controllers_.emplace(joint->name, pid);
          this->pid_controllers_[joint->name].init(kVelPidGainsNh);

          initVelocityCommandHandle(joint);
          continue;
        }
      }

      if (transmission.type_ == "franka_hw/FrankaStateInterface") {
        ROS_INFO_STREAM_NAMED("franka_hw_sim",
                              "Found transmission interface '" << transmission.type_ << "'");
        try {
          initFrankaStateHandle(this->arm_id_, *urdf, transmission);
          continue;

        } catch (const std::invalid_argument& e) {
          ROS_ERROR_STREAM_NAMED("franka_hw_sim", e.what());
          return false;
        }
      }

      if (transmission.type_ == "franka_hw/FrankaModelInterface") {
        ROS_INFO_STREAM_NAMED("franka_hw_sim",
                              "Found transmission interface '" << transmission.type_ << "'");
        double singularity_threshold;
        model_nh.param<double>("singularity_warning_threshold", singularity_threshold, -1);
        try {
          initFrankaModelHandle(this->arm_id_, *urdf, transmission, singularity_threshold);
          continue;

        } catch (const std::invalid_argument& e) {
          ROS_ERROR_STREAM_NAMED("franka_hw_sim", e.what());
          return false;
        }
      }
      ROS_WARN_STREAM_NAMED("franka_hw_sim", "Unsupported transmission interface of joint '"
                                                 << joint->name << "': " << k_interface);
    }
  }

  // After all handles have been assigned to interfaces, register them
  registerInterface(&this->eji_);
  registerInterface(&this->pji_);
  registerInterface(&this->vji_);
  registerInterface(&this->jsi_);
  registerInterface(&this->fsi_);
  registerInterface(&this->fmi_);

  // Initialize ROS Services
  initServices(model_nh);

  // Connect to '/gazebo/set_model_configuration' service
  this->gazebo_set_model_configuration_client_ =
      model_nh.serviceClient<gazebo_msgs::SetModelConfiguration>("/gazebo/set_model_configuration");

  return readParameters(model_nh, *urdf);
}

void FrankaHWSim::initJointStateHandle(const std::shared_ptr<franka_gazebo::Joint>& joint) {
  this->jsi_.registerHandle(hardware_interface::JointStateHandle(joint->name, &joint->position,
                                                                 &joint->velocity, &joint->effort));
}

void FrankaHWSim::initEffortCommandHandle(const std::shared_ptr<franka_gazebo::Joint>& joint) {
  this->eji_.registerHandle(
      hardware_interface::JointHandle(this->jsi_.getHandle(joint->name), &joint->command));
}

void FrankaHWSim::initPositionCommandHandle(const std::shared_ptr<franka_gazebo::Joint>& joint) {
  this->pji_.registerHandle(
      hardware_interface::JointHandle(this->jsi_.getHandle(joint->name), &joint->command));
}

void FrankaHWSim::initVelocityCommandHandle(const std::shared_ptr<franka_gazebo::Joint>& joint) {
  this->vji_.registerHandle(
      hardware_interface::JointHandle(this->jsi_.getHandle(joint->name), &joint->command));
}

void FrankaHWSim::initFrankaStateHandle(
    const std::string& robot,
    const urdf::Model& urdf,
    const transmission_interface::TransmissionInfo& transmission) {
  if (transmission.joints_.size() != 7) {
    throw std::invalid_argument(
        "Cannot create franka_hw/FrankaStateInterface for robot '" + robot + "_robot' because " +
        std::to_string(transmission.joints_.size()) +
        " joints were found beneath the <transmission> tag, but 7 are required.");
  }

  // Check if all joints defined in the <transmission> actually exist in the URDF
  for (const auto& joint : transmission.joints_) {
    if (not urdf.getJoint(joint.name_)) {
      throw std::invalid_argument("Cannot create franka_hw/FrankaStateInterface for robot '" +
                                  robot + "_robot' because the specified joint '" + joint.name_ +
                                  "' in the <transmission> tag cannot be found in the URDF");
    }
    ROS_DEBUG_STREAM_NAMED("franka_hw_sim",
                           "Found joint " << joint.name_ << " to belong to a Panda robot");
  }
  this->fsi_.registerHandle(franka_hw::FrankaStateHandle(robot + "_robot", this->robot_state_));
}

void FrankaHWSim::initFrankaModelHandle(
    const std::string& robot,
    const urdf::Model& urdf,
    const transmission_interface::TransmissionInfo& transmission,
    double singularity_threshold) {
  if (transmission.joints_.size() != 2) {
    throw std::invalid_argument(
        "Cannot create franka_hw/FrankaModelInterface for robot '" + robot + "_model' because " +
        std::to_string(transmission.joints_.size()) +
        " joints were found beneath the <transmission> tag, but 2 are required.");
  }

  for (const auto& joint : transmission.joints_) {
    if (not urdf.getJoint(joint.name_)) {
      if (not urdf.getJoint(joint.name_)) {
        throw std::invalid_argument("Cannot create franka_hw/FrankaModelInterface for robot '" +
                                    robot + "_model' because the specified joint '" + joint.name_ +
                                    "' in the <transmission> tag cannot be found in the URDF");
      }
    }
  }
  auto root =
      std::find_if(transmission.joints_.begin(), transmission.joints_.end(),
                   [&](const transmission_interface::JointInfo& i) { return i.role_ == "root"; });
  if (root == transmission.joints_.end()) {
    throw std::invalid_argument("Cannot create franka_hw/FrankaModelInterface for robot '" + robot +
                                "_model' because no <joint> with <role>root</root> can be found "
                                "in the <transmission>");
  }
  auto tip =
      std::find_if(transmission.joints_.begin(), transmission.joints_.end(),
                   [&](const transmission_interface::JointInfo& i) { return i.role_ == "tip"; });
  if (tip == transmission.joints_.end()) {
    throw std::invalid_argument("Cannot create franka_hw/FrankaModelInterface for robot '" + robot +
                                "_model' because no <joint> with <role>tip</role> can be found "
                                "in the <transmission>");
  }
  try {
    auto root_link = urdf.getJoint(root->name_)->parent_link_name;
    auto tip_link = urdf.getJoint(tip->name_)->child_link_name;

    this->model_ =
        std::make_unique<franka_gazebo::ModelKDL>(urdf, root_link, tip_link, singularity_threshold);

  } catch (const std::invalid_argument& e) {
    throw std::invalid_argument("Cannot create franka_hw/FrankaModelInterface for robot '" + robot +
                                "_model'. " + e.what());
  }
  this->fmi_.registerHandle(
      franka_hw::FrankaModelHandle(robot + "_model", *this->model_, this->robot_state_));
}

void FrankaHWSim::initServices(ros::NodeHandle& nh) {
  this->service_set_ee_ =
      nh.advertiseService<franka_msgs::SetEEFrame::Request, franka_msgs::SetEEFrame::Response>(
          "set_EE_frame", [&](auto& request, auto& response) {
            ROS_INFO_STREAM_NAMED("franka_hw_sim",
                                  this->arm_id_ << ": Setting NE_T_EE transformation");
            std::copy(request.NE_T_EE.cbegin(), request.NE_T_EE.cend(),
                      this->robot_state_.NE_T_EE.begin());
            this->updateRobotStateDynamics();
            response.success = true;
            return true;
          });
  this->service_set_k_ = franka_hw::advertiseService<franka_msgs::SetKFrame>(
      nh, "set_K_frame", [&](auto& request, auto& response) {
        ROS_INFO_STREAM_NAMED("franka_hw_sim", this->arm_id_ << ": Setting EE_T_K transformation");
        std::copy(request.EE_T_K.cbegin(), request.EE_T_K.cend(),
                  this->robot_state_.EE_T_K.begin());
        this->updateRobotStateDynamics();
        response.success = true;
        return true;
      });
  this->service_set_load_ = franka_hw::advertiseService<franka_msgs::SetLoad>(
      nh, "set_load", [&](auto& request, auto& response) {
        ROS_INFO_STREAM_NAMED("franka_hw_sim", this->arm_id_ << ": Setting Load");
        this->robot_state_.m_load = request.mass;
        std::copy(request.F_x_center_load.cbegin(), request.F_x_center_load.cend(),
                  this->robot_state_.F_x_Cload.begin());
        std::copy(request.load_inertia.cbegin(), request.load_inertia.cend(),
                  this->robot_state_.I_load.begin());
        this->updateRobotStateDynamics();
        response.success = true;
        return true;
      });
  this->service_collision_behavior_ =
      franka_hw::advertiseService<franka_msgs::SetForceTorqueCollisionBehavior>(
          nh, "set_force_torque_collision_behavior", [&](auto& request, auto& response) {
            ROS_INFO_STREAM_NAMED("franka_hw_sim", this->arm_id_ << ": Setting Collision Behavior");

            for (int i = 0; i < 7; i++) {
              std::string name = this->arm_id_ + "_joint" + std::to_string(i + 1);
              this->joints_[name]->contact_threshold =
                  request.lower_torque_thresholds_nominal.at(i);
              this->joints_[name]->collision_threshold =
                  request.upper_torque_thresholds_nominal.at(i);
            }

            std::move(request.lower_force_thresholds_nominal.begin(),
                      request.lower_force_thresholds_nominal.end(),
                      this->lower_force_thresholds_nominal_.begin());
            std::move(request.upper_force_thresholds_nominal.begin(),
                      request.upper_force_thresholds_nominal.end(),
                      this->upper_force_thresholds_nominal_.begin());

            response.success = true;
            return true;
          });
  this->service_set_model_configuration_ =
      franka_hw::advertiseService<franka_msgs::SetJointConfiguration>(
          nh, "set_franka_model_configuration", [&](auto& request, auto& response) {
            // Check if positions equals the number of joints
            if (request.configuration.name.size() == 1 && request.configuration.name[0].empty()) {
              ROS_ERROR_STREAM_NAMED("franka_hw_sim",
                                     "Setting of Franka model configuration failed since "
                                     "no joints were specified in the request.");
              response.success = false;
              response.error = "no joints specified";
              return false;
            }
            if (request.configuration.name.size() != request.configuration.position.size()) {
              ROS_ERROR_STREAM_NAMED("franka_hw_sim",
                                     "Setting of Franka model configuration failed since "
                                     "the 'position' and 'name' fields were of unequal length.");
              response.success = false;
              response.error = "'position' and 'name' fields length unequal";
              return false;
            }

            // Print request information
            std::string requested_configuration_string;
            for (int ii = 0; ii < request.configuration.name.size(); ii++) {
                requested_configuration_string += request.configuration.name[ii] + ": " + std::to_string(request.configuration.position[ii])  + ", ";
            }
            requested_configuration_string = requested_configuration_string.substr(0, requested_configuration_string.length() - 2); //Remove last 2 characters
            ROS_INFO_STREAM_NAMED("franka_hw_sim", "Setting joint configuration: " << requested_configuration_string);

            // Validate joint names and joint limits
            std::map<std::string, double> model_configuration;
            for (int ii = 0; ii < request.configuration.name.size(); ii++) {
              std::string joint(request.configuration.name[ii]);
              double position(request.configuration.position[ii]);

              if (this->joints_.find(joint) != this->joints_.end()) { // If joint exists
                double min_position(this->joints_[joint]->limits.min_position);
                double max_position(this->joints_[joint]->limits.max_position);

                // Check if position is within joint limits
                if (position == boost::algorithm::clamp(position, min_position, max_position)) {
                  model_configuration.emplace(std::make_pair(joint, position));
                } else {
                  ROS_WARN_STREAM_NAMED("franka_hw_sim",
                                        "Joint configuration of joint '"
                                            << joint << "' was not set since the requested joint position ("
                                            << position << ") is not within joint limits (i.e. "
                                            << min_position << " - " << max_position << ").");
                }
              } else {
                ROS_WARN_STREAM_NAMED("franka_hw_sim",
                                      "Joint configuration of joint '"
                                          << joint
                                          << "' not set since it is not a valid panda joint.");
              }
            }

            // Return if no valid positions were found
            if (model_configuration.size() == 0){
              ROS_ERROR_STREAM_NAMED("franka_hw_sim",
                        "Setting of Franka model configuration aborted since no valid "
                        "joint configuration were found.");
              response.success = false;
              response.error = "no valid joint configurations";
              return false;
            }

            // Throw warnings about unused request fields
            if (request.configuration.effort.size() != 0) {
              ROS_WARN_STREAM_ONCE_NAMED("franka_hw_sim",
                                    "The 'set_franka_model_configuration' service does not use the "
                                    "'effort' field.");
            }
            if (request.configuration.velocity.size() != 0) {
              ROS_WARN_STREAM_ONCE_NAMED("franka_hw_sim",
                                    "The 'set_franka_model_configuration' service does not use the "
                                    "'velocity' field.");
            }

            // Call Gazebo 'set_model_configuration' service and update Franka joint positions
            gazebo_msgs::SetModelConfiguration gazebo_model_configuration;
            gazebo_model_configuration.request.model_name = "panda";
            for (const auto& pair : model_configuration) {
              gazebo_model_configuration.request.joint_names.push_back(pair.first);
              gazebo_model_configuration.request.joint_positions.push_back(pair.second);
            }
            if (this->gazebo_set_model_configuration_client_.call(gazebo_model_configuration)) {
              // Update franka positions
              for (const auto& pair : model_configuration) {
                this->joints_[pair.first]->setJointPosition(pair.second);
              }

              response.success = true;
              return true;
            } else {
              ROS_WARN_STREAM_NAMED(
                  "franka_hw_sim",
                  "Setting of Franka model configuration failed since "
                  "a problem occurred when setting the joint configuration in Gazebo.");
              response.success = false;
              return false;
            }
          });
}

void FrankaHWSim::readSim(ros::Time time, ros::Duration period) {
  for (const auto& pair : this->joints_) {
    auto joint = pair.second;
    joint->update(period);
  }
  this->updateRobotState(time);
}

void FrankaHWSim::writeSim(ros::Time /*time*/, ros::Duration period) {
  auto g = this->model_->gravity(this->robot_state_, this->gravity_earth_);

  for (auto& pair : this->joints_) {
    auto joint = pair.second;

    // Retrieve effort control command
    double effort;
    switch (joint_control_methods_[joint->name]) {
      case EFFORT: {
        // Check if this joint is affected by gravity compensation
        std::string prefix = this->arm_id_ + "_joint";
        if (pair.first.rfind(prefix, 0) != std::string::npos) {
          int i = std::stoi(pair.first.substr(prefix.size())) - 1;
          joint->gravity = g.at(i);
        }
        effort = joint->command + joint->gravity;
      } break;
      case POSITION: {
        // Use position motion generator
        double error;
        const double kJointLowerLimit = joint->limits.min_position;
        const double kJointUpperLimit = joint->limits.max_position;
        switch (joint->type) {
          case urdf::Joint::REVOLUTE:
            angles::shortest_angular_distance_with_limits(
                joint->position, joint->command, kJointLowerLimit, kJointUpperLimit, error);
            break;
          case urdf::Joint::CONTINUOUS:
            error = angles::shortest_angular_distance(joint->position, joint->command);
            break;
          default:
            error = joint->command - joint->position;
        }
        const double kEffortLimit = joint->limits.max_effort;
        effort =
            boost::algorithm::clamp(pid_controllers_[joint->name].computeCommand(error, period),
                                    -kEffortLimit, kEffortLimit);
      } break;
      case VELOCITY: {
        // Use velocity motion generator
        const double kError = joint->command - joint->velocity;
        const double kEffortLimit = joint->limits.max_effort;
        effort =
            boost::algorithm::clamp(pid_controllers_[joint->name].computeCommand(kError, period),
                                    -kEffortLimit, kEffortLimit);
      } break;
    }

    // Send control effort control command
    if (std::isnan(effort)) {
      ROS_WARN_STREAM_NAMED("franka_hw_sim",
                            "Command for " << joint->name << "is NaN, won't send to robot");
      continue;
    }
    joint->handle->SetForce(0, effort);
  }
}

void FrankaHWSim::eStopActive(bool /* active */) {}

bool FrankaHWSim::readParameters(const ros::NodeHandle& nh, const urdf::Model& urdf) {
  try {
    guessEndEffector(nh, urdf);

    nh.param<double>("m_load", this->robot_state_.m_load, 0);

    std::string I_load;  // NOLINT [readability-identifier-naming]
    nh.param<std::string>("I_load", I_load, "0 0 0 0 0 0 0 0 0");
    this->robot_state_.I_load = readArray<9>(I_load, "I_load");

    std::string F_x_Cload;  // NOLINT [readability-identifier-naming]
    nh.param<std::string>("F_x_Cload", F_x_Cload, "0 0 0");
    this->robot_state_.F_x_Cload = readArray<3>(F_x_Cload, "F_x_Cload");

    std::string NE_T_EE;  // NOLINT [readability-identifier-naming]
    nh.param<std::string>("NE_T_EE", NE_T_EE, "1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1");
    this->robot_state_.NE_T_EE = readArray<16>(NE_T_EE, "NE_T_EE");

    std::string EE_T_K;  // NOLINT [readability-identifier-naming]
    nh.param<std::string>("EE_T_K", EE_T_K, "1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1");
    this->robot_state_.EE_T_K = readArray<16>(EE_T_K, "EE_T_K");

    std::string gravity_vector;
    if (nh.getParam("gravity_vector", gravity_vector)) {
      this->gravity_earth_ = readArray<3>(gravity_vector, "gravity_vector");
    }

    // Only nominal cases supported for now
    std::vector<double> lower_torque_thresholds = franka_hw::FrankaHW::getCollisionThresholds(
        "lower_torque_thresholds_nominal", nh, {20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0});

    std::vector<double> upper_torque_thresholds = franka_hw::FrankaHW::getCollisionThresholds(
        "upper_torque_thresholds_nominal", nh, {20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0});

    this->lower_force_thresholds_nominal_ = franka_hw::FrankaHW::getCollisionThresholds(
        "lower_torque_thresholds_nominal", nh, {20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0});
    this->upper_force_thresholds_nominal_ = franka_hw::FrankaHW::getCollisionThresholds(
        "upper_torque_thresholds_nominal", nh, {20.0, 20.0, 20.0, 25.0, 25.0, 25.0});

    for (int i = 0; i < 7; i++) {
      std::string name = this->arm_id_ + "_joint" + std::to_string(i + 1);
      this->joints_[name]->contact_threshold = lower_torque_thresholds.at(i);
      this->joints_[name]->collision_threshold = upper_torque_thresholds.at(i);
    }

  } catch (const std::invalid_argument& e) {
    ROS_ERROR_STREAM_NAMED("franka_hw_sim", e.what());
    return false;
  }
  updateRobotStateDynamics();
  return true;
}

void FrankaHWSim::guessEndEffector(const ros::NodeHandle& nh, const urdf::Model& urdf) {
  auto hand_link = this->arm_id_ + "_hand";
  auto hand = urdf.getLink(hand_link);
  if (hand != nullptr) {
    ROS_INFO_STREAM_NAMED("franka_hw_sim",
                          "Found link '" << hand_link
                                         << "' in URDF. Assuming it is defining the kinematics & "
                                            "inertias of a Franka Hand Gripper.");
  }

  // By absolute default unless URDF or ROS params say otherwise, assume no end-effector.
  double def_m_ee = 0;
  std::string def_i_ee = "0.0 0 0 0 0.0 0 0 0 0.0";
  std::string def_f_x_cee = "0 0 0";
  std::string def_f_t_ne = "1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1";
  if (not nh.hasParam("F_T_NE") and hand != nullptr) {
    // NOTE: We cannot interprete the Joint pose from the URDF directly, because
    // its <arm_id>_link is mounted at the flange directly and not at NE
    def_f_t_ne = "0.7071 -0.7071 0 0 0.7071 0.7071 0 0 0 0 1 0 0 0 0.1034 1";
  }
  std::string F_T_NE;  // NOLINT [readability-identifier-naming]
  nh.param<std::string>("F_T_NE", F_T_NE, def_f_t_ne);
  this->robot_state_.F_T_NE = readArray<16>(F_T_NE, "F_T_NE");

  if (not nh.hasParam("m_ee") and hand != nullptr) {
    if (hand->inertial == nullptr) {
      throw std::invalid_argument("Trying to use inertia of " + hand_link +
                                  " but this link has no <inertial> tag defined in it.");
    }
    def_m_ee = hand->inertial->mass;
  }
  nh.param<double>("m_ee", this->robot_state_.m_ee, def_m_ee);

  if (not nh.hasParam("I_ee") and hand != nullptr) {
    if (hand->inertial == nullptr) {
      throw std::invalid_argument("Trying to use inertia of " + hand_link +
                                  " but this link has no <inertial> tag defined in it.");
    }
    // clang-format off
    def_i_ee = std::to_string(hand->inertial->ixx) + " " + std::to_string(hand->inertial->ixy) + " " + std::to_string(hand->inertial->ixz) + " "
             + std::to_string(hand->inertial->ixy) + " " + std::to_string(hand->inertial->iyy) + " " + std::to_string(hand->inertial->iyz) + " "
             + std::to_string(hand->inertial->ixz) + " " + std::to_string(hand->inertial->iyz) + " " + std::to_string(hand->inertial->izz);
    // clang-format on
  }
  std::string I_ee;  // NOLINT [readability-identifier-naming]
  nh.param<std::string>("I_ee", I_ee, def_i_ee);
  this->robot_state_.I_ee = readArray<9>(I_ee, "I_ee");

  if (not nh.hasParam("F_x_Cee") and hand != nullptr) {
    if (hand->inertial == nullptr) {
      throw std::invalid_argument("Trying to use inertia of " + hand_link +
                                  " but this link has no <inertial> tag defined in it.");
    }
    def_f_x_cee = std::to_string(hand->inertial->origin.position.x) + " " +
                  std::to_string(hand->inertial->origin.position.y) + " " +
                  std::to_string(hand->inertial->origin.position.z);
  }
  std::string F_x_Cee;  // NOLINT [readability-identifier-naming]
  nh.param<std::string>("F_x_Cee", F_x_Cee, def_f_x_cee);
  this->robot_state_.F_x_Cee = readArray<3>(F_x_Cee, "F_x_Cee");
}

void FrankaHWSim::updateRobotStateDynamics() {
  this->robot_state_.m_total = this->robot_state_.m_ee + this->robot_state_.m_load;

  Eigen::Map<Eigen::Matrix4d>(this->robot_state_.F_T_EE.data()) =
      Eigen::Matrix4d(this->robot_state_.F_T_NE.data()) *
      Eigen::Matrix4d(this->robot_state_.NE_T_EE.data());

  Eigen::Map<Eigen::Matrix3d>(this->robot_state_.I_total.data()) =
      shiftInertiaTensor(Eigen::Matrix3d(this->robot_state_.I_ee.data()), this->robot_state_.m_ee,
                         Eigen::Vector3d(this->robot_state_.F_x_Cload.data()));
}

void FrankaHWSim::updateRobotState(ros::Time time) {
  // This is ensured, because a FrankaStateInterface checks for at least seven joints in the URDF
  assert(this->joints_.size() >= 7);

  for (int i = 0; i < 7; i++) {
    std::string name = this->arm_id_ + "_joint" + std::to_string(i + 1);
    const auto& joint = this->joints_.at(name);
    this->robot_state_.q[i] = joint->position;
    this->robot_state_.dq[i] = joint->velocity;
    this->robot_state_.tau_J[i] = joint->effort;
    this->robot_state_.dtau_J[i] = joint->jerk;

    this->robot_state_.q_d[i] = joint->position;
    this->robot_state_.dq_d[i] = joint->velocity;
    this->robot_state_.ddq_d[i] = joint->acceleration;
    this->robot_state_.tau_J_d[i] = joint->command;

    // For now we assume no flexible joints
    this->robot_state_.theta[i] = joint->position;
    this->robot_state_.dtheta[i] = joint->velocity;

    if (this->efforts_initialized_) {
      // NOTE: Here we use the clamped command to filter out the internal controller
      // force when the joint is in its limits.
      double tau_ext = joint->effort - joint->clamped_command + joint->gravity;

      // Exponential moving average filter from tau_ext -> tau_ext_hat_filtered
      this->robot_state_.tau_ext_hat_filtered[i] =
          this->tau_ext_lowpass_filter_ * tau_ext +
          (1 - this->tau_ext_lowpass_filter_) * this->robot_state_.tau_ext_hat_filtered[i];
    }

    this->robot_state_.joint_contact[i] = static_cast<double>(joint->isInContact());
    this->robot_state_.joint_collision[i] = static_cast<double>(joint->isInCollision());
  }

  // Calculate estimated wrenches in Task frame from external joint torques with jacobians
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_ext(this->robot_state_.tau_ext_hat_filtered.data());
  Eigen::MatrixXd j0_transpose_pinv;
  Eigen::MatrixXd jk_transpose_pinv;
  Eigen::Matrix<double, 6, 7> j0(
      this->model_->zeroJacobian(franka::Frame::kStiffness, this->robot_state_).data());
  Eigen::Matrix<double, 6, 7> jk(
      this->model_->bodyJacobian(franka::Frame::kStiffness, this->robot_state_).data());
  franka_example_controllers::pseudoInverse(j0.transpose(), j0_transpose_pinv);
  franka_example_controllers::pseudoInverse(jk.transpose(), jk_transpose_pinv);

  Eigen::VectorXd f_ext_0 = j0_transpose_pinv * tau_ext;
  Eigen::VectorXd f_ext_k = jk_transpose_pinv * tau_ext;
  Eigen::VectorXd::Map(&this->robot_state_.O_F_ext_hat_K[0], 6) = f_ext_0;
  Eigen::VectorXd::Map(&this->robot_state_.K_F_ext_hat_K[0], 6) = f_ext_k;

  for (int i = 0; i < this->robot_state_.cartesian_contact.size(); i++) {
    // Evaluate the cartesian contacts/collisions in K frame
    double fi = std::abs(f_ext_k(i));
    this->robot_state_.cartesian_contact[i] =
        static_cast<double>(fi > this->lower_force_thresholds_nominal_.at(i));
    this->robot_state_.cartesian_collision[i] =
        static_cast<double>(fi > this->upper_force_thresholds_nominal_.at(i));
  }

  this->robot_state_.control_command_success_rate = 1.0;
  this->robot_state_.time = franka::Duration(time.toNSec() / 1e6 /*ms*/);
  this->robot_state_.O_T_EE = this->model_->pose(franka::Frame::kEndEffector, this->robot_state_);

  this->efforts_initialized_ = true;
}

}  // namespace franka_gazebo

PLUGINLIB_EXPORT_CLASS(franka_gazebo::FrankaHWSim, gazebo_ros_control::RobotHWSim)
