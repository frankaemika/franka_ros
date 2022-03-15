#pragma once

#include <angles/angles.h>
#include <joint_limits_interface/joint_limits.h>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <gazebo/physics/Joint.hh>

namespace franka_gazebo {

/**
 * Specifies the current control method of the joint.
 */
enum ControlMethod { EFFORT, POSITION, VELOCITY };

/**
 * A data container holding all relevant information about a robotic joint.
 *
 * Calling @ref update on this object will compute its internal state based on the all currently
 * supplied information such as position, efforts etc.
 */
struct Joint {
 public:
  Joint() = default;
  Joint(Joint&&) = default;
  Joint(const Joint&) = delete;

  /**
   * Calculate all members such as accelerations, jerks velocities by differentiation
   * @param[in] dt the current time step since last time this method was called
   */
  void update(const ros::Duration& dt);

  /// Name of this joint. Should be unique in whole simulation
  std::string name;

  /// A pointer to the underlying gazebo handle. Must be non-null for update to work
  gazebo::physics::JointPtr handle;

  /// The type of joint, i.e. revolute, prismatic, ... @see
  /// http://docs.ros.org/en/diamondback/api/urdf/html/classurdf_1_1Joint.html
  int type;

  /// Joint limits @see
  /// https://docs.ros.org/en/diamondback/api/urdf/html/classurdf_1_1JointLimits.html
  joint_limits_interface::JointLimits limits;

  /// The axis of rotation/translation of this joint in local coordinates
  Eigen::Vector3d axis;

  /// The currently applied command from a controller acting on this joint either in \f$N\f$ or
  /// \f$Nm\f$ without gravity
  double command = 0;

  /// The current desired position that is used for the PID controller when the joints control
  /// method is "POSITION". When the control method is not "POSITION", this value will only be
  /// updated once at the start of the controller and stay the same until a new controller is
  /// started.
  double desired_position = 0;

  /// The current desired velocity that is used for the PID controller when the joints control
  /// method is "VELOCITY". When the control method is not "VELOCITY", this value will be set to
  /// zero.
  double desired_velocity = 0;

  /// Decides whether the joint is doing torque control or if the position or velocity should
  /// be controlled.
  ControlMethod control_method = POSITION;

  /// The currently acting gravity force or torque acting on this joint in \f$N\f$ or \f$Nm\f$
  double gravity = 0;

  /// The current position of this joint either in \f$m\f$ or \f$rad\f$
  double position = 0;

  /// The current velocity of this joint either in \f$\frac{m}{s}\f$ or \f$\frac{rad}{s}\f$
  double velocity = 0;

  /// The current total force or torque acting on this joint in either \f$N\f$ or \f$Nm\f$
  double effort = 0;

  /// The currently acting jerk acting on this this joint in either \f$\frac{m}{s^3}\f$ or
  /// \f$\frac{rad}{s^3}\f$
  double jerk = 0;

  /// The currently acting acceleration on this joint in either \f$\frac{m}{s^2}\f$ or
  /// \f$\frac{rad}{s^2}\f$
  double acceleration = 0;

  /// Above which threshold forces or torques will be interpreted as "contacts" by @ref isInContact
  double contact_threshold = std::numeric_limits<double>::infinity();

  /// Above which threshold forces or torques will be interpreted as "collisions" by @ref
  /// isInCollision
  double collision_threshold = std::numeric_limits<double>::infinity();

  /**
   * Get the total link mass of this joint's child
   * @return the mass in \f$kg\f$
   */
  double getLinkMass() const;

  /**
   * Is the joint currently in contact with something?
   * @return `true` if @ref effort > @ref contact_threshold
   */
  bool isInContact() const;

  /**
   * Is the joint currently in contact with something?
   * @return `true` if @ref effort > @ref collision_threshold
   */
  bool isInCollision() const;

 private:
  double lastVelocity = std::numeric_limits<double>::quiet_NaN();
  double lastAcceleration = std::numeric_limits<double>::quiet_NaN();
};

}  // namespace franka_gazebo
