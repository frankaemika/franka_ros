#include <franka_gazebo/joint.h>
#include <urdf/model.h>
#include <gazebo/physics/Link.hh>

namespace franka_gazebo {

void Joint::update(const ros::Duration& dt) {
  if (not this->handle) {
    return;
  }

  this->velocity = this->handle->GetVelocity(0);
  double position = this->handle->Position(0);
  ignition::math::Vector3d f;
  switch (this->type) {
    case urdf::Joint::PRISMATIC:
      this->position = position;
      f = this->handle->GetForceTorque(0).body2Force;
      break;
    case urdf::Joint::REVOLUTE:
    case urdf::Joint::CONTINUOUS:
      this->position += angles::shortest_angular_distance(this->position, position);
      f = this->handle->GetForceTorque(0).body2Torque;
      break;
  }
  this->effort = Eigen::Vector3d(f.X(), f.Y(), f.Z()).dot(this->axis);

  // TODO filter derivatives? Maybe make it configurable?
  if (std::isnan(this->lastVelocity)) {
    this->lastVelocity = this->velocity;
  }
  this->acceleration = (this->velocity - this->lastVelocity) / dt.toSec();
  this->lastVelocity = this->velocity;

  if (std::isnan(this->lastAcceleration)) {
    this->lastAcceleration = this->acceleration;
  }
  this->jerk = (this->acceleration - this->lastAcceleration) / dt.toSec();
  this->lastAcceleration = this->acceleration;
}

double Joint::getLinkMass() const {
  if (not this->handle) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return this->handle->GetChild()->GetInertial()->Mass();
}

bool Joint::isInCollision() const {
  return this->effort - this->command > this->collision_threshold;
}

bool Joint::isInContact() const {
  return this->effort - this->command > this->contact_threshold;
}
}  // namespace franka_gazebo
