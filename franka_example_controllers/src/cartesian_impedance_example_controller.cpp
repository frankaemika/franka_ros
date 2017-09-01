#include <franka_example_controllers/cartesian_impedance_example_controller.h>

#include <cmath>

#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka/robot_state.h>

#include <franka_hw/SetForceTorqueCollisionBehavior.h>

namespace franka_example_controllers {

CartesianImpedanceExampleController::CartesianImpedanceExampleController() = default;

bool CartesianImpedanceExampleController::init(hardware_interface::RobotHW* robot_hw,
                                           ros::NodeHandle& node_handle) {
  std::vector<double> cartesian_stiffness_vector;
  std::vector<double> cartesian_damping_vector;
  node_handle_ = node_handle;

  sub_equilibrium_pose_ = node_handle_.subscribe("/equilibrium_pose",20,
                                                 &CartesianImpedanceExampleController::equilibrium_pose_callback,
                                                 this, ros::TransportHints().reliable().tcpNoDelay());

  if (!node_handle_.getParam("arm_id", arm_id_)) {
    ROS_ERROR_STREAM("CartesianImpedanceExampleController: Could not read parameter arm_id");
    return false;
  }
  if (!node_handle_.getParam("pos_error_max", pos_error_max_)) {
    ROS_INFO_STREAM(
        "CartesianImpedanceExampleController: No parameter pos_error_max, defaulting to: " << pos_error_max_);
  }
  if (!node_handle_.getParam("rot_error_max", rot_error_max_)) {
    ROS_INFO_STREAM(
        "CartesianImpedanceExampleController: No parameter rot_error_max, defaulting to: " << rot_error_max_);
  }
  if (!node_handle_.getParam("k_p_nullspace", k_p_nullspace_)) {
    ROS_INFO_STREAM(
        "CartesianImpedanceExampleController: No parameter k_p_nullspace, defaulting to: " << k_p_nullspace_);
  }
  if (!node_handle_.getParam("k_ext", k_ext_)) {
    ROS_INFO_STREAM(
        "CartesianImpedanceExampleController: No parameter k_ext, defaulting to: " << k_ext_);
  }
  if (!node_handle_.getParam("filter_gain", filter_gain_)) {
    ROS_INFO_STREAM(
        "CartesianImpedanceExampleController: No parameter filter_gain, defaulting to: " << filter_gain_);
  }
  if (!node_handle_.getParam("joint_names", joint_names_) || joint_names_.size() != 7) {
    ROS_ERROR(
        "CartesianImpedanceExampleController: Invalid or no joint_names parameters provided, aborting "
        "controller init!");
    return false;
  }

  franka_hw::FrankaModelInterface* model_interface =
      robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceExampleController: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_.reset(
        new franka_hw::FrankaModelHandle(model_interface->getHandle(arm_id_ + "_model")));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceExampleController: Exception getting model handle from interface: "
            << ex.what());
    return false;
  }

  franka_hw::FrankaStateInterface* state_interface =
      robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceExampleController: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_.reset(new franka_hw::FrankaStateHandle(
        state_interface->getHandle(arm_id_ + "_robot")));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceExampleController: Exception getting state handle from interface: "
            << ex.what());
    return false;
  }

  hardware_interface::EffortJointInterface* effort_joint_interface =
      robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceExampleController: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names_[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "CartesianImpedanceExampleController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  dynamic_reconfigure_compliance_param_node_ = ros::NodeHandle("dynamic_reconfigure_compliance_param_node");

  dynamic_server_compliance_param_.reset(
      new dynamic_reconfigure::Server<
        franka_example_controllers::compliance_paramConfig>(dynamic_reconfigure_compliance_param_node_));
  dynamic_server_compliance_param_->setCallback(
      boost::bind(&CartesianImpedanceExampleController::compliance_param_callback, this, _1, _2));

  position_d_.setZero();
  orientation_d_.coeffs() << 0.0, 0.0, 0.0, 1.0;
  dx_.setZero();

  // Set force/torque collision behavior
//  ros::ServiceClient client = node_handle_.serviceClient<franka_hw::SetForceTorqueCollisionBehavior> (
//      "/" + arm_id_ + "/set_force_torque_collision_behavior");
//  franka_hw::SetForceTorqueCollisionBehavior collision_srv;
//  collision_srv.request.lower_torque_thresholds_nominal = {{70.0, 70.0, 70.0, 70.0, 70.0, 70.0, 70.0}};
//  collision_srv.request.upper_torque_thresholds_nominal = {{70.0, 70.0, 70.0, 70.0, 70.0, 70.0, 70.0}};
//  collision_srv.request.lower_force_thresholds_nominal = {{70.0, 70.0, 70.0, 70.0, 70.0, 70.0}};
//  collision_srv.request.upper_force_thresholds_nominal = {{70.0, 70.0, 70.0, 70.0, 70.0, 70.0}};
//  if (!client.call(collision_srv)) {
//    ROS_ERROR_STREAM("Failed to call set_force_torque_collision_behavior service: " + collision_srv.response.error);
//  }

  return true;
}

void CartesianImpedanceExampleController::starting(const ros::Time& /*time*/) {
  /* Compute initial velocity with jacobian and set x_attractor and q_d_nullspace to initial configuration*/
  franka::RobotState initial_state = state_handle_->getRobotState();
  // get jacobian
  std::array<double, 42> jacobian_array = model_handle_->getZeroJacobian(franka::Frame::kEndEffector, initial_state);
  // convert to eigen
  Eigen::Map<Eigen::Matrix<double, 6, 7> > jacobian(jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1> > dq_initial(initial_state.dq.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1> > q_initial(initial_state.q.data());
  Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));

  // Set equilibrium point to current state
  position_d_ = initial_transform.translation();
  orientation_d_ = Eigen::Quaterniond(initial_transform.rotation());

  // Set nullspace equilibrium configuration to current state
  q_d_nullspace_ = q_initial;

  // Initial twist
  dx_ << jacobian * dq_initial;
}

void CartesianImpedanceExampleController::update(const ros::Time& /*time*/,
                                             const ros::Duration& period) {

  // Get state variables
  franka::RobotState robot_state = state_handle_->getRobotState();
  std::array<double, 49> inertia_array = model_handle_->getMass(
      {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}}, 0.0, {{0.0, 0.0, 0.0}});
  std::array<double, 7> coriolis_array = model_handle_->getCoriolis(
      {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}}, 0.0, {{0.0, 0.0, 0.0}});
  std::array<double, 42> jacobian_array = model_handle_->getZeroJacobian(
      franka::Frame::kEndEffector, robot_state);

  // Convert to Eigen
  Eigen::Map<Eigen::Matrix<double, 7, 1> > coriolis(coriolis_array.data());
  Eigen::Map<Eigen::Matrix<double, 6, 7> > jacobian(jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 7> > inertia(inertia_array.data());

  Eigen::Map<Eigen::Matrix<double, 7, 1> > q(robot_state.q.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1> > dq(robot_state.dq.data());
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Vector3d position(transform.translation());
  Eigen::Quaterniond orientation(transform.rotation());

  /*** Compute cartesian mass matrix ***/
  // Compute cartesian mass inverting J * M(q)^{-1} J^T with SVD
  Eigen::EigenSolver<Eigen::MatrixXd> es(jacobian * inertia.inverse() * jacobian.transpose(), true);
  Eigen::VectorXd eigenvals = es.eigenvalues().real();
  // Ensure positive definiteness
  for (size_t e=1 ; e < 6 ; e++) {
    if (eigenvals(e) < 1e-20) {
      eigenvals(e) = 1e-20;
    }
  }
  Eigen::Matrix<double, 6, 6> cartesian_mass = es.eigenvectors().real() *
      eigenvals.asDiagonal().inverse() * es.eigenvectors().real().transpose();

  /*** Exponential smoother to filter twist signal ***/
  dx_ = (1.0 - filter_gain_) * dx_ + filter_gain_ * (jacobian * dq);

  /*** compute error to desired pose ***/
  /* position error */
  Eigen::Matrix<double, 6, 1> error;
  error.head(3) << position - position_d_;

  /* orientation error */
  // make sure the error is computed from the "closest" quaternion
  if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0) {
    orientation.coeffs() << - orientation.coeffs();
  }
  // "difference" quaternion
  Eigen::Quaterniond error_quaternion(orientation * orientation_d_.inverse());
  // normalize (to make sure the quaternion is well defined)
  error_quaternion.coeffs() << error_quaternion.coeffs() / error_quaternion.coeffs().norm();
  // convert to axis angle
  Eigen::AngleAxisd error_quaternion_angle_axis(error_quaternion);
  // compute "orientation error"
  error.tail(3) << error_quaternion_angle_axis.axis() * error_quaternion_angle_axis.angle();
  // saturate position error
  if (error.head(3).norm() > pos_error_max_) {
    error.head(3) << (error.head(3) / error.head(3).norm()) * pos_error_max_;
  }
  // saturate rotation error
  if (error.tail(3).norm() > rot_error_max_) {
    error.tail(3) << (error.tail(3) / error.tail(3).norm()) * rot_error_max_;
  }

  /*** Compute control ***/
  // Allocate variables (for clarity)
  Eigen::VectorXd tau_task(7), tau_nullspace(7);

  /* Pseudoinverse for nullspace handling */
  // Kinematic pseuoinverse
  Eigen::MatrixXd jacobian_transpose_pinv;
  pseudo_inverse(jacobian.transpose(), jacobian_transpose_pinv);
  // Dynamically consistent "pseudoinverse" (Khatib)
  // Needs high gains and becomes too noisy due to inversions
  // jacobian_transpose_pinv = cartesian_mass * jacobian * inertia.inverse();

  tau_task = jacobian.transpose() * cartesian_mass * (- cartesian_stiffness_ * error - cartesian_damping_ * dx_);
  tau_nullspace = (Eigen::MatrixXd::Identity(7, 7) - jacobian.transpose()*jacobian_transpose_pinv)
      * (k_p_nullspace_ * (q_d_nullspace_ - q) - (2.0 * sqrt(k_p_nullspace_)) * dq);

  std::array<double, 7> tau_d;
  for (size_t i = 0; i < 7; ++i) {
    tau_d[i] = tau_task(i) + tau_nullspace(i) + coriolis(i)
        - k_ext_ * robot_state.tau_ext_hat_filtered[i]; // "reduces" mass at a joint level
    joint_handles_[i].setCommand(tau_d[i]);
  }
  return;
}

void CartesianImpedanceExampleController::compliance_param_callback(
    franka_example_controllers::compliance_paramConfig& config, uint32_t level) {
  cartesian_stiffness_.setIdentity();
  cartesian_stiffness_.topLeftCorner(3, 3) <<
    config.translational_stiffness * Eigen::Matrix3d::Identity();
  cartesian_stiffness_.bottomRightCorner(3, 3) <<
    config.rotational_stiffness * Eigen::Matrix3d::Identity();
  cartesian_damping_.setIdentity();
  cartesian_damping_.topLeftCorner(3, 3) <<
    config.translational_damping * Eigen::Matrix3d::Identity();
  cartesian_damping_.bottomRightCorner(3, 3) <<
    config.rotational_damping * Eigen::Matrix3d::Identity();
}

void CartesianImpedanceExampleController::equilibrium_pose_callback(const geometry_msgs::PoseStampedConstPtr &msg) {
  position_d_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
  orientation_d_.coeffs() << msg->pose.orientation.x, msg->pose.orientation.y,
    msg->pose.orientation.z, msg->pose.orientation.w;
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::CartesianImpedanceExampleController,
                       controller_interface::ControllerBase)
