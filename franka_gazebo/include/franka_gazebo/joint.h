#pragma once

#include <angles/angles.h>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <gazebo/physics/Joint.hh>

namespace franka_gazebo {

struct Joint {
 public:
  Joint() = default;
  Joint(Joint&&) = default;
  Joint(const Joint&) = delete;

  void update(const ros::Duration& dt);

  std::string name;
  gazebo::physics::JointPtr handle;
  int type;
  Eigen::Vector3d axis;
  double command;
  double position;
  double velocity;
  double effort;
  double jerk;
  double acceleration;
  double contact_threshold = std::numeric_limits<double>::infinity();
  double collision_threshold = std::numeric_limits<double>::infinity();

  double getLinkMass() const;
  bool isInContact() const;
  bool isInCollision() const;

 private:
  double lastVelocity = std::numeric_limits<double>::quiet_NaN();
  double lastEffort = std::numeric_limits<double>::quiet_NaN();
};

}  // namespace franka_gazebo
