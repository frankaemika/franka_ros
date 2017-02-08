#include <franka/robot.h>

#include <franka_hw/franka_hw.h>
#include <pluginlib/class_list_macros.h>

#include <math.h>  // floor()
#include <string>

franka_hw::FrankaHW::FrankaHW() {}

franka_hw::FrankaHW::~FrankaHW() {
  // TODO what to call to shutdown conntection to robot??
  delete robot_;
}

/**
* \param nh A private node_handle needed to parse parameters like joint_names
* and Franka robot IP etc.
*/
void franka_hw::FrankaHW::init(const ros::NodeHandle& nh) {
  // parse robot_hw yaml with joint names and robot-IP:
  XmlRpc::XmlRpcValue params;
  nh.getParam("joint_names", params);
  joint_name_.resize(params.size());
  for (int i = 0; i < params.size(); ++i) {
    joint_name_[i] = static_cast<std::string>(params[i]);
    ROS_INFO("joint %d: %s", i, joint_name_.at(i).c_str());
  }
  nh.getParam("robot_ip", robot_ip_);

  setUpRobot(robot_ip_);

  // resize members for state storage
  q_.resize(joint_name_.size());
  dq_.resize(joint_name_.size());
  q_d_.resize(joint_name_.size());
  q_start_.resize(joint_name_.size());
  tau_J_.resize(joint_name_.size());
  dtau_J_.resize(joint_name_.size());
  tau_ext_hat_filtered_.resize(joint_name_.size());
  cartesian_collision_.resize(6);
  cartesian_contact_.resize(6);
  joint_collision_.resize(joint_name_.size());
  joint_contact_.resize(joint_name_.size());
  EE_F_ext_hat_EE_.resize(6);
  O_F_ext_hat_EE_.resize(6);
  elbow_start_.resize(2);
  O_T_EE_start_.resize(0);
  O_T_EE_start_.push_back(std::vector<double>{0.0, 0.0, 0.0, 0.0});
  O_T_EE_start_.push_back(std::vector<double>{0.0, 0.0, 0.0, 0.0});
  O_T_EE_start_.push_back(std::vector<double>{0.0, 0.0, 0.0, 0.0});
  O_T_EE_start_.push_back(std::vector<double>{0.0, 0.0, 0.0, 0.0});

  // register joint handles:
  for (size_t i = 0; i < joint_name_.size(); ++i) {
    // connect the standard joint state interface
    hardware_interface::JointStateHandle jnt_handle1(joint_name_[i], &q_[i],
                                                     &dq_[i], &tau_J_[i]);
    jnt_state_interface_.registerHandle(jnt_handle1);

    // connect the franka joint state interface
    hardware_interface::FrankaJointStateHandle jnt_handle2(
        joint_name_[i], &q_[i], &dq_[i], &tau_J_[i], &q_d_[i], &q_start_[i],
        &dtau_J_[i], &tau_ext_hat_filtered_[i], &joint_collision_[i],
        &joint_contact_[i]);
    franka_jnt_state_interface_.registerHandle(jnt_handle2);
  }

  // register cartesian handle:
  hardware_interface::FrankaCartesianStateHandle cart_handle(
      std::string("franka_emika_cartesian_data"), &cartesian_collision_,
      &cartesian_contact_, &O_F_ext_hat_EE_, &EE_F_ext_hat_EE_, &O_T_EE_start_);
  franka_cart_state_interface_.registerHandle(cart_handle);

  // register interfaces:
  registerInterface(&jnt_state_interface_);
  registerInterface(&franka_jnt_state_interface_);
  registerInterface(&franka_cart_state_interface_);
}

bool franka_hw::FrankaHW::update() {
  try {
    // read from franka
    if (robot_->waitForRobotState()) {
      // write data and to members
      updateStates(robot_->robotState());
      // publishFrankaStates();
      // publishJointStates();
      return true;
    } else {
      ROS_ERROR_THROTTLE(1, "failed to read franka state ");
      return false;
    }
  } catch (franka::NetworkException const& e) {
    std::cout << e.what() << std::endl;
    return false;
  }
}

/**
* \param robot_state A data struct for franka robot states as received with
* libfranka
*/
void franka_hw::FrankaHW::updateStates(const franka::RobotState& robot_state) {
  for (size_t i = 0; i < q_.size(); ++i) {
    q_.at(i) = robot_state.q[i];
    dq_.at(i) = robot_state.dq[i];
    q_d_.at(i) = robot_state.q_d[i];
    q_start_.at(i) = robot_state.q_start[i];
    tau_J_.at(i) = robot_state.tau_J[i];
    dtau_J_.at(i) = robot_state.dtau_J[i];
    tau_ext_hat_filtered_.at(i) = robot_state.tau_ext_hat_filtered[i];
    joint_collision_.at(i) = robot_state.joint_collision[i];
    joint_contact_.at(i) = robot_state.joint_contact[i];
  }

  for (size_t i = 0; i < cartesian_collision_.size(); ++i) {
    cartesian_collision_.at(i) = robot_state.cartesian_collision[i];
    cartesian_contact_.at(i) = robot_state.cartesian_contact[i];
  }

  for (size_t i = 0; i < EE_F_ext_hat_EE_.size(); ++i) {
    EE_F_ext_hat_EE_.at(i) = robot_state.EE_F_ext_hat_EE[i];
    O_F_ext_hat_EE_.at(i) = robot_state.O_F_ext_hat_EE[i];
  }

  for (size_t i = 0; i < elbow_start_.size(); ++i) {
    elbow_start_.at(i) = robot_state.elbow_start[i];
  }

  for (size_t i = 0; i < O_T_EE_start_.size(); ++i) {
    size_t row = floor(i / 4);
    size_t col = i % 4;
    O_T_EE_start_[row][col] = robot_state.O_T_EE_start[i];
  }
}

void franka_hw::FrankaHW::publishFrankaStates() {
  // TODO
}

void franka_hw::FrankaHW::publishJointStates() {
  // TODO
}

/**
* \param ip The IP address of the franka emika robot to connect to with
* libfranka
*/
bool franka_hw::FrankaHW::setUpRobot(std::string ip) {
  try {
    robot_ = new franka::Robot(ip.c_str());
    return true;
  } catch (franka::NetworkException const& e) {
    std::cout << e.what() << std::endl;
    return false;
  }
}

std::string franka_hw::FrankaHW::getRobotIp() const {
  return robot_ip_;
}

PLUGINLIB_EXPORT_CLASS(franka_hw::FrankaHW, hardware_interface::RobotHW)
