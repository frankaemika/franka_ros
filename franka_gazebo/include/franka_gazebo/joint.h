#pragma once

#include <angles/angles.h>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <gazebo/physics/Joint.hh>

namespace franka_gazebo {

/**
 * A data container holding all relevant information about a robotic joint.
 *
 * Calling @ref update on this object will compute its internal state based on the all currenlty
 * supplied information such as position, efforts etc.
 */
struct Joint {
 public:
  Joint() = default;
  Joint(Joint&&) = default;
  Joint(const Joint&) = delete;

  /**
   * Calculate all members such as accelerations, jerks velocities by differention
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

  /// The axis of rotation/translation of this joint in local coordinates
  Eigen::Vector3d axis;

  /// The currently applied command from an actuator on this joint either in \f$N\f$ or \f$Nm\f$
  double command = 0;

  /// The current position of this joint either in \f$m\f$ or \f$rad\f$
  double position = 0;

  /// The current velocity of this joint either in \f$\frac{m}{s}\f$ or \f$\frac{rad}{s}\f$
  double velocity = 0;

  /// The current total force or torque acting on this joint in either \f$N\f$ or \f$Nm\f$
  double effort = 0;

  /// The currently acting jerk acting on this this joint in either \f$\frac{m}{s^3}\f$ or
  /// \f$\frac{rad}{s^3}\f$
  double jerk = 0;

  /// The currenlty acting acceleration on this joint in either \f$\frac{m}{s^2}\f$ or
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
