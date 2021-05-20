#include <franka_gazebo/model_kdl.h>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>
#include <kdl/solveri.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

namespace franka_gazebo {

int ModelKDL::segment(franka::Frame frame) {
  // clang-format off
  switch (frame) {
    case franka::Frame::kJoint1: return 1;
    case franka::Frame::kJoint2: return 2;
    case franka::Frame::kJoint3: return 3;
    case franka::Frame::kJoint4: return 4;
    case franka::Frame::kJoint5: return 5;
    case franka::Frame::kJoint6: return 6;
    case franka::Frame::kJoint7: return 7;
    case franka::Frame::kFlange: return 8;
    case franka::Frame::kEndEffector:
    case franka::Frame::kStiffness:
      return -1;
  }
  // clang-format on
}

// Implementation copied from <kdl/isolveri.hpp> because
// KDL::ChainDynSolver inherits *privately* from SolverI ... -.-'
std::string ModelKDL::strError(const int error) {
  // clang-format off
  switch(error) {
  case KDL::SolverI::E_NOERROR:                 return "No error"; break;
  case KDL::SolverI::E_NO_CONVERGE:             return "Failed to converge"; break;
  case KDL::SolverI::E_UNDEFINED:               return "Undefined value"; break;
  case KDL::SolverI::E_DEGRADED:                return "Converged but degraded solution"; break;
  case KDL::SolverI::E_NOT_UP_TO_DATE:          return "Internal data structures not up to date with Chain"; break;
  case KDL::SolverI::E_SIZE_MISMATCH:           return "The size of the input does not match the internal state"; break;
  case KDL::SolverI::E_MAX_ITERATIONS_EXCEEDED: return "The maximum number of iterations is exceeded"; break;
  case KDL::SolverI::E_OUT_OF_RANGE:            return "The requested index is out of range"; break;
  case KDL::SolverI::E_NOT_IMPLEMENTED:         return "The requested function is not yet implemented"; break;
  case KDL::SolverI::E_SVD_FAILED:              return "SVD failed"; break;
  default: return "UNKNOWN ERROR";
  }
  // clang-format on
}
ModelKDL::ModelKDL(const urdf::Model& urdf, std::string root, std::string tip) {
  KDL::Tree tree;
  if (not kdl_parser::treeFromUrdfModel(urdf, tree)) {
    throw std::invalid_argument("Cannot contruct KDL tree from URDF");
  }

  if (not tree.getChain(root, tip, this->chain_)) {
    throw std::invalid_argument("Cannot find chain within URDF tree from root '" + root +
                                "' to tip '" + tip + "'. Do these links exist?");
  }

  ROS_INFO_STREAM("KDL Model initialized for chain from '" << root << "' -> '" << tip << "'");

  // TODO check if zero grav vector works. It seems unused anyways in the impl
  this->dynamicsSolver_ = std::make_unique<KDL::ChainDynParam>(chain_, KDL::Vector(0, 0, -9.81));
  this->jacobianSolver_ = std::make_unique<KDL::ChainJntToJacSolver>(chain_);
  this->kinematicsSolver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain_);
}

std::array<double, 16> ModelKDL::pose(
    franka::Frame frame,
    const std::array<double, 7>& q,
    const std::array<double, 16>& F_T_EE,  // NOLINT(readability-identifier-naming)
    const std::array<double, 16>& EE_T_K)  // NOLINT(readability-identifier-naming)
    const {
  KDL::JntArray kq;
  KDL::Frame kp;
  kq.data = Eigen::Matrix<double, 7, 1>(q.data());

  int error = this->kinematicsSolver_->JntToCart(kq, kp, segment(frame));
  if (error != KDL::SolverI::E_NOERROR) {
    throw std::logic_error("KDL forward kinematics pose calculation failed with error: " +
                           strError(error));
  }
  Eigen::Matrix4d p;
  // clang-format off
  p <<
      kp(0,0), kp(0,1), kp(0,2), kp(0,3),
      kp(1,0), kp(1,1), kp(1,2), kp(1,3),
      kp(2,0), kp(2,1), kp(2,2), kp(2,3),
      kp(3,0), kp(3,1), kp(3,2), kp(3,3);
  // clang-format on

  std::array<double, 16> result;
  Eigen::MatrixXd::Map(&result[0], 4, 4) = p;

  return result;
}

std::array<double, 42> ModelKDL::bodyJacobian(
    franka::Frame frame,
    const std::array<double, 7>& q,
    const std::array<double, 16>& F_T_EE,  // NOLINT(readability-identifier-naming)
    const std::array<double, 16>& EE_T_K)  // NOLINT(readability-identifier-naming)
    const {
  throw std::runtime_error("Not implemented: bodyJacobian()");
}

std::array<double, 42> ModelKDL::zeroJacobian(
    franka::Frame frame,
    const std::array<double, 7>& q,
    const std::array<double, 16>& F_T_EE,  // NOLINT(readability-identifier-naming)
    const std::array<double, 16>& EE_T_K)  // NOLINT(readability-identifier-naming)
    const {
  KDL::JntArray kq;
  KDL::Jacobian J(7);
  kq.data = Eigen::Matrix<double, 7, 1>(q.data());

  int error = this->jacobianSolver_->JntToJac(kq, J, segment(frame));
  if (error != KDL::SolverI::E_NOERROR) {
    throw std::logic_error("KDL zero jacobian calculation failed with error: " + strError(error));
  }

  std::array<double, 42> result;
  Eigen::MatrixXd::Map(&result[0], 6, 7) = J.data;

  return result;
}

std::array<double, 49> ModelKDL::mass(
    const std::array<double, 7>& q,
    const std::array<double, 9>& I_total,  // NOLINT(readability-identifier-naming)
    double m_total,
    const std::array<double, 3>& F_x_Ctotal)  // NOLINT(readability-identifier-naming)
    const {
  KDL::JntArray kq;
  KDL::JntSpaceInertiaMatrix M(7);
  kq.data = Eigen::Matrix<double, 7, 1>(q.data());

  int error = this->dynamicsSolver_->JntToMass(kq, M);
  if (error != KDL::SolverI::E_NOERROR) {
    throw std::logic_error("KDL mass calculation failed with error: " + strError(error));
  }

  std::array<double, 49> result;
  Eigen::MatrixXd::Map(&result[0], 7, 7) = M.data;

  return result;
}

std::array<double, 7> ModelKDL::coriolis(
    const std::array<double, 7>& q,
    const std::array<double, 7>& dq,
    const std::array<double, 9>& I_total,  // NOLINT(readability-identifier-naming)
    double m_total,
    const std::array<double, 3>& F_x_Ctotal)  // NOLINT(readability-identifier-naming)
    const {
  KDL::JntArray kq, kdq, kc(7);
  kq.data = Eigen::Matrix<double, 7, 1>(q.data());
  kdq.data = Eigen::Matrix<double, 7, 1>(dq.data());

  int error = this->dynamicsSolver_->JntToCoriolis(kq, kdq, kc);
  if (error != KDL::SolverI::E_NOERROR) {
    throw std::logic_error("KDL coriolis calculation failed with error: " + strError(error));
  }

  std::array<double, 7> result;
  Eigen::VectorXd::Map(&result[0], kc.data.size()) = kc.data;

  return result;
}

std::array<double, 7> ModelKDL::gravity(
    const std::array<double, 7>& q,
    double m_total,
    const std::array<double, 3>& F_x_Ctotal,  // NOLINT(readability-identifier-naming)
    const std::array<double, 3>& gravity_earth) const {
  KDL::JntArray kq, kg(7);
  kq.data = Eigen::Matrix<double, 7, 1>(q.data());

  int error = this->dynamicsSolver_->JntToGravity(kq, kg);
  if (error != KDL::SolverI::E_NOERROR) {
    throw std::logic_error("KDL gravity calculation failed with error: " + strError(error));
  }

  std::array<double, 7> result;
  Eigen::VectorXd::Map(&result[0], kg.data.size()) = kg.data;

  return result;
}
}  // namespace franka_gazebo
