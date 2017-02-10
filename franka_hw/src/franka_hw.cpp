#include <franka/robot.h>

#include <franka_hw/FrankaState.h>
#include <franka_hw/franka_hw.h>

#include <sensor_msgs/JointState.h>
#include <std_msgs/MultiArrayDimension.h>

#include <pluginlib/class_list_macros.h>

#include <math.h>    // floor()
#include <iostream>  // std::cout
#include <string>

franka_hw::FrankaHW::FrankaHW() {}

franka_hw::FrankaHW::~FrankaHW() {
    // TODO what to call to shutdown conntection to robot??
    delete robot_;
}

/**
* \param nh A private node_handle needed to parse parameters like joint_names,
* the Franka robot IP etc.
*/
void franka_hw::FrankaHW::init(const ros::NodeHandle& nh) {
    // parse robot_hw yaml with joint names and robot-IP:
    XmlRpc::XmlRpcValue params;
    nh.getParam("joint_names", params);
    joint_name_.resize(params.size());
    for (int i = 0; i < params.size(); ++i) {
        joint_name_[i] = static_cast<std::string>(params[i]);
        ROS_INFO("parsed name of joint %d: %s", i, joint_name_[i].c_str());
    }
    nh.getParam("robot_ip", robot_ip_);

    setUpRobot(robot_ip_);

    // resize members for state storage:
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
        hardware_interface::FrankaJointStateHandle jnt_handle2(joint_name_[i],
                                                               &q_[i],
                                                               &dq_[i],
                                                               &tau_J_[i],
                                                               &q_d_[i],
                                                               &q_start_[i],
                                                               &dtau_J_[i],
                                                               &tau_ext_hat_filtered_[i],
                                                               &joint_collision_[i],
                                                               &joint_contact_[i]);
        franka_jnt_state_interface_.registerHandle(jnt_handle2);
    }

    // register cartesian handle:
    hardware_interface::FrankaCartesianStateHandle cart_handle(std::string("franka_emika_cartesian_data"),
                                                               &cartesian_collision_,
                                                               &cartesian_contact_,
                                                               &O_F_ext_hat_EE_,
                                                               &EE_F_ext_hat_EE_,
                                                               &O_T_EE_start_);

    // connect cartesian interface
    franka_cart_state_interface_.registerHandle(cart_handle);

    // register all interfaces:
    registerInterface(&jnt_state_interface_);
    registerInterface(&franka_jnt_state_interface_);
    registerInterface(&franka_cart_state_interface_);

    // register realtime publishers:
    pub_franka_states_ =
            new realtime_tools::RealtimePublisher<franka_hw::FrankaState>(nh, "franka_states", 1);
    pub_joint_states_ =
            new realtime_tools::RealtimePublisher<sensor_msgs::JointState>(nh, "joint_states", 1);

    // resize publisher messages:
    pub_franka_states_->msg_.cartesian_collision.resize(
                cartesian_collision_.size());
    pub_franka_states_->msg_.cartesian_contact.resize(cartesian_contact_.size());
    pub_franka_states_->msg_.dq.resize(dq_.size());
    pub_franka_states_->msg_.dtau_J.resize(dtau_J_.size());
    pub_franka_states_->msg_.EE_F_ext_hat_EE.resize(EE_F_ext_hat_EE_.size());
    pub_franka_states_->msg_.elbow_start.resize(elbow_start_.size());
    pub_franka_states_->msg_.joint_collision.resize(joint_collision_.size());
    pub_franka_states_->msg_.joint_contact.resize(joint_contact_.size());
    pub_franka_states_->msg_.O_F_ext_hat_EE.resize(O_F_ext_hat_EE_.size());
    pub_franka_states_->msg_.q.resize(q_.size());
    pub_franka_states_->msg_.q_d.resize(q_d_.size());
    pub_franka_states_->msg_.q_start.resize(q_start_.size());
    pub_franka_states_->msg_.tau_ext_hat_filtered.resize(
                tau_ext_hat_filtered_.size());
    pub_franka_states_->msg_.tau_J.resize(tau_J_.size());

    pub_franka_states_->msg_.O_T_EE_start.layout.data_offset = 0;
    pub_franka_states_->msg_.O_T_EE_start.layout.dim.clear();
    pub_franka_states_->msg_.O_T_EE_start.layout.dim.push_back(std_msgs::MultiArrayDimension());
    pub_franka_states_->msg_.O_T_EE_start.layout.dim[0].size = 4;
    pub_franka_states_->msg_.O_T_EE_start.layout.dim[0].stride = 4*4;
    pub_franka_states_->msg_.O_T_EE_start.layout.dim[0].label = "row";
    pub_franka_states_->msg_.O_T_EE_start.layout.dim.push_back(std_msgs::MultiArrayDimension());
    pub_franka_states_->msg_.O_T_EE_start.layout.dim[1].size = 4;
    pub_franka_states_->msg_.O_T_EE_start.layout.dim[1].stride = 4;
    pub_franka_states_->msg_.O_T_EE_start.layout.dim[1].label = "column";
    pub_franka_states_->msg_.O_T_EE_start.data.resize(16);

    pub_joint_states_->msg_.name.resize(joint_name_.size());
    pub_joint_states_->msg_.position.resize(q_.size());
    pub_joint_states_->msg_.velocity.resize(dq_.size());
    pub_joint_states_->msg_.effort.resize(tau_J_.size());
}

bool franka_hw::FrankaHW::update() {
    try {
        // read from franka
        if (robot_->waitForRobotState()) {
            // write data to members/state_interfaces:
            updateStates(robot_->robotState());

            // publish from members:
            publishFrankaStates();
            publishJointStates();
            return true;
        } else {
            ROS_ERROR_THROTTLE(1, "failed to read franka state");
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
        q_[i] = robot_state.q[i];
        dq_[i] = robot_state.dq[i];
        q_d_[i] = robot_state.q_d[i];
        q_start_[i] = robot_state.q_start[i];
        tau_J_[i] = robot_state.tau_J[i];
        dtau_J_[i] = robot_state.dtau_J[i];
        tau_ext_hat_filtered_[i] = robot_state.tau_ext_hat_filtered[i];
        joint_collision_[i] = robot_state.joint_collision[i];
        joint_contact_[i] = robot_state.joint_contact[i];
    }

    for (size_t i = 0; i < cartesian_collision_.size(); ++i) {
        cartesian_collision_[i] = robot_state.cartesian_collision[i];
        cartesian_contact_[i] = robot_state.cartesian_contact[i];
    }

    for (size_t i = 0; i < EE_F_ext_hat_EE_.size(); ++i) {
        EE_F_ext_hat_EE_[i] = robot_state.EE_F_ext_hat_EE[i];
        O_F_ext_hat_EE_[i] = robot_state.O_F_ext_hat_EE[i];
    }

    for (size_t i = 0; i < elbow_start_.size(); ++i) {
        elbow_start_[i] = robot_state.elbow_start[i];
    }

    for (size_t i = 0; i < robot_state.O_T_EE_start.size(); ++i) {
        size_t col = floor(i / 4);
        size_t row = i % 4;
        O_T_EE_start_[row][col] = robot_state.O_T_EE_start[i];
    }
}

void franka_hw::FrankaHW::publishFrankaStates() {
    if (pub_franka_states_->trylock()) {
        try {  // fill message with data:
            for (size_t i = 0; i < cartesian_collision_.size(); ++i) {
                pub_franka_states_->msg_.cartesian_collision[i] = cartesian_collision_[i];
                pub_franka_states_->msg_.cartesian_contact[i] = cartesian_contact_[i];
                pub_franka_states_->msg_.EE_F_ext_hat_EE[i] = EE_F_ext_hat_EE_[i];
                pub_franka_states_->msg_.O_F_ext_hat_EE[i] = O_F_ext_hat_EE_[i];
            }

            for (size_t i = 0; i < q_.size(); ++i) {
                pub_franka_states_->msg_.q[i] = q_[i];
                pub_franka_states_->msg_.dq[i] = dq_[i];
                pub_franka_states_->msg_.tau_J[i] = tau_J_[i];
                pub_franka_states_->msg_.dtau_J[i] = dtau_J_[i];
                pub_franka_states_->msg_.joint_collision[i] = joint_collision_[i];
                pub_franka_states_->msg_.joint_contact[i] = joint_contact_[i];
                pub_franka_states_->msg_.q_d[i] = q_d_[i];
                pub_franka_states_->msg_.q_start[i] = q_start_[i];
                pub_franka_states_->msg_.tau_ext_hat_filtered[i] = tau_ext_hat_filtered_[i];
            }

            for (size_t i = 0; i < elbow_start_.size(); ++i) {
                pub_franka_states_->msg_.elbow_start[i] = elbow_start_[i];
            }

            for (size_t row = 0; row < 4; ++row) {
                for (size_t col = 0; col < 4; ++col) {
                    pub_franka_states_->msg_.O_T_EE_start.data[4*row + col] = O_T_EE_start_[row][col];
                }
            }
        } catch (const std::out_of_range& e) {
            std::cout << "Out of Range error." << e.what();
            pub_franka_states_->unlock();
            return;
        }

        pub_franka_states_->msg_.header.seq = seq_nr_fra_;
        pub_franka_states_->msg_.header.stamp = ros::Time::now();
        pub_franka_states_->unlockAndPublish();
        seq_nr_fra_++;
    } else  // could not lock franka_states to publish
    {
        missed_pulishes_franka_++;
        ROS_WARN("could not lock franka_states for publishing, missed %i of %i",
                 int(missed_pulishes_franka_), int(seq_nr_fra_));
        seq_nr_fra_++;  // package loss can be detected when seq_nr jumps
    }
}

void franka_hw::FrankaHW::publishJointStates() {
    if (pub_joint_states_->trylock()) {
        try {

            for (size_t i = 0; i < joint_name_.size(); ++i) {
                pub_joint_states_->msg_.name[i] = joint_name_[i];
                pub_joint_states_->msg_.position[i] = q_[i];
                pub_joint_states_->msg_.velocity[i] = dq_[i];
                pub_joint_states_->msg_.effort[i] = tau_J_[i];
            }
        }
        catch (const std::out_of_range& e) {
            std::cout << "Out of Range error." << e.what();
            pub_joint_states_->unlock();
            return;
        }
        pub_joint_states_->msg_.header.stamp = ros::Time::now();
        pub_joint_states_->msg_.header.seq = seq_nr_jnt_;
        pub_joint_states_->unlockAndPublish();
        seq_nr_jnt_++;
    }
    else {
        missed_pulishes_joint_++;
        ROS_WARN("could not lock joint_states for publishing, missed %i of %i",
                 int(missed_pulishes_joint_), int(seq_nr_jnt_));
        seq_nr_jnt_++;  // package loss can be detected when seq_nr jumps
    }
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
