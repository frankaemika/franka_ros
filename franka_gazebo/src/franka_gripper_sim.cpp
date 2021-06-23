#include <memory>

#include <franka_gazebo/franka_gripper_sim.h>
#include <pluginlib/class_list_macros.h>
#include <cmath>

namespace franka_gazebo {

using actionlib::SimpleActionServer;
using control_msgs::GripperCommandAction;
using franka_gripper::GraspAction;
using franka_gripper::HomingAction;
using franka_gripper::MoveAction;
using franka_gripper::StopAction;

bool FrankaGripperSim::init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& nh) {
  std::string ns = nh.getNamespace();
  std::string finger1, finger2;
  if (not nh.getParam("finger1/joint", finger1)) {
    ROS_ERROR_STREAM_NAMED("FrankaGripperSim",
                           "Could not find required parameter '" << ns << "/finger1/joint'");
    return false;
  }
  if (not nh.getParam("finger2/joint", finger2)) {
    ROS_ERROR_STREAM_NAMED("FrankaGripperSim",
                           "Could not find required parameter '" << ns << "/finger2/joint'");
    return false;
  }

  nh.param<double>("move/width_tolerance", this->tolerance_move_, 0.005);
  nh.param<double>("gripper_action/width_tolerance", this->tolerance_gripper_action_, 0.005);
  nh.param<double>("gripper_action/speed", this->speed_default_, 0.1);
  nh.param<double>("grasp/resting_threshold", this->speed_threshold_, 0.005);
  nh.param<int>("grasp/consecutive_samples", this->speed_samples_, 3);

  try {
    this->finger1_ = hw->getHandle(finger1);
    this->finger2_ = hw->getHandle(finger2);
  } catch (const hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM_NAMED("FrankaGripperSim", "Could not get joint handle(s): " << ex.what());
    return false;
  }

  if (not this->pid1_.initParam(ns + "/finger1/gains")) {
    return false;
  }
  if (not this->pid2_.initParam(ns + "/finger2/gains")) {
    return false;
  }

  pub_.init(nh, "joint_states", 1);
  pub_.msg_.name = {finger1, finger2};
  ROS_INFO_STREAM_NAMED("FrankaGripperSim",
                        "Successfully initialized Franka Gripper Controller for joints '"
                            << finger1 << "' and '" << finger2 << "'");

  this->action_stop_ = std::make_unique<SimpleActionServer<StopAction>>(
      nh, "stop",
      [&](auto&& goal) {
        franka_gripper:
          ROS_INFO_STREAM_NAMED("FrankaGripperSim", "Stop Action goal received");

          this->interrupt("Command interrupted, because stop action was called", State::IDLE);

          {
            std::lock_guard<std::mutex> lock(this->mutex_);
            this->force_desired_ = 0;
            this->speed_desired_ = 0;
            this->state_ = State::IDLE;
          }

          franka_gripper::StopResult result;
          result.success = static_cast<decltype(result.success)>(true);
          action_stop_->setSucceeded(result);
      },
      false);
  this->action_stop_->start();

  this->action_homing_ = std::make_unique<SimpleActionServer<HomingAction>>(
      nh, "homing",
      [&](auto&& goal) {
        ROS_INFO_STREAM_NAMED("FrankaGripperSim", "New Homing Action goal received");

        if (this->state_ != State::IDLE) {
          this->interrupt("Command interrupted, because new homing action called", State::HOMING);
        }

        {
          std::lock_guard<std::mutex> lock(this->mutex_);
          this->mutex_.lock();
          this->width_desired_ = 0;
          this->speed_desired_ = -0.02;
          this->force_desired_ = 0;
          this->tolerance_.inner = this->tolerance_move_;
          this->tolerance_.outer = this->tolerance_move_;
          this->state_ = State::HOMING;
        }

        this->waitUntil(State::IDLE);

        franka_gripper::HomingResult result;
        result.success = static_cast<decltype(result.success)>(true);
        action_homing_->setSucceeded(result);
      },
      false);
  this->action_homing_->registerPreemptCallback([&]() {
    ROS_INFO_STREAM_NAMED("FrankaGripperSim", "Homing Action cancelled");
    std::lock_guard<std::mutex> lock(this->mutex_);
    this->state_ = State::IDLE;
  });
  this->action_homing_->start();

  this->action_move_ = std::make_unique<SimpleActionServer<MoveAction>>(
      nh, "move",
      [&](auto&& goal) {
        ROS_INFO_STREAM_NAMED("FrankaGripperSim",
                              "New Move Action Goal received: " << goal->width << " m");
        if (goal->speed < 0) {
          franka_gripper::MoveResult result;
          result.success = static_cast<decltype(result.success)>(false);
          result.error = "Only positive speeds allowed";
          action_move_->setAborted(result, result.error);
          return;
        }

        if (goal->width < 0 or goal->width > kMaxFingerWidth) {
          franka_gripper::MoveResult result;
          result.success = static_cast<decltype(result.success)>(false);
          result.error = "Target width has to lie between 0 .. " + std::to_string(kMaxFingerWidth);
          action_move_->setAborted(result, result.error);
          return;
        }

        if (this->state_ != State::IDLE) {
          this->interrupt("Command interrupted, because new move action called", State::MOVING);
        }

        {
          std::lock_guard<std::mutex> lock(this->mutex_);
          this->width_desired_ = goal->width;
          this->speed_desired_ = goal->speed;
          this->force_desired_ = 0;
          this->tolerance_.inner = this->tolerance_move_;
          this->tolerance_.outer = this->tolerance_move_;
          this->state_ = State::MOVING;
        }

        this->waitUntil(State::IDLE);

        franka_gripper::MoveResult result;
        result.success = static_cast<decltype(result.success)>(true);
        action_move_->setSucceeded(result);
      },
      false);
  this->action_move_->registerPreemptCallback([&]() {
    ROS_INFO_STREAM_NAMED("FrankaGripperSim", "Moving Action cancelled");
    std::lock_guard<std::mutex> lock(this->mutex_);
    this->state_ = State::IDLE;
  });
  this->action_move_->start();

  this->action_grasp_ = std::make_unique<SimpleActionServer<GraspAction>>(
      nh, "grasp",
      [&](auto&& goal) {
        ROS_INFO_STREAM_NAMED("FrankaGripperSim",
                              "New Grasp Action Goal received: " << goal->force << "N");

        if (goal->speed < 0) {
          franka_gripper::GraspResult result;
          result.success = static_cast<decltype(result.success)>(false);
          result.error = "Only positive speeds allowed";
          action_grasp_->setAborted(result, result.error);
          return;
        }

        if (this->state_ != State::IDLE) {
          this->interrupt("Command interrupted, because new grasp action called", State::GRASPING);
        }

        double width = this->finger1_.getPosition() + this->finger2_.getPosition();
        {
          std::lock_guard<std::mutex> lock(this->mutex_);
          // Don't use goal as desired width, because we have might have to go beyond until contact
          this->width_desired_ = goal->width < width ? 0 : kMaxFingerWidth;
          this->speed_desired_ = goal->speed;
          this->force_desired_ = goal->force;
          this->tolerance_ = goal->epsilon;
          this->state_ = State::GRASPING;
        }

        this->waitUntil(State::HOLDING);

        width = this->finger1_.getPosition() + this->finger2_.getPosition();  // recalculate
        franka_gripper::GraspResult result;
        bool ok =
            goal->width - goal->epsilon.inner < width and width < goal->width + goal->epsilon.outer;
        result.success = static_cast<decltype(result.success)>(ok);
        double speed = this->finger1_.getVelocity() + this->finger2_.getVelocity();
        if (not ok) {
          result.error =
              "When the gripper stopped (below speed of " + std::to_string(this->speed_threshold_) +
              " m/s the width between the fingers was not at " + std::to_string(goal->width) +
              "m (-" + std::to_string(goal->epsilon.inner) + "m/+" +
              std::to_string(goal->epsilon.outer) + "m) but at " + std::to_string(width) + "m";
          action_grasp_->setAborted(result, result.error);

          std::lock_guard<std::mutex> lock(this->mutex_);
          this->state_ = State::IDLE;
        } else {
          action_grasp_->setSucceeded(result);
        }
      },
      false);
  this->action_grasp_->registerPreemptCallback([&]() {
    ROS_INFO_STREAM_NAMED("FrankaGripperSim", "Grasping Action cancelled");
    std::lock_guard<std::mutex> lock(this->mutex_);
    this->state_ = State::IDLE;
  });
  this->action_grasp_->start();

  this->action_gc_ = std::make_unique<SimpleActionServer<GripperCommandAction>>(
      nh, "gripper_action",
      [&](auto&& goal) {
        ROS_INFO_STREAM_NAMED("FrankaGripperSim", "New Gripper Command Action Goal received: "
                                                      << goal->command.max_effort << "N");

        // HACK: As one gripper finger is <mimic>, MoveIt!'s trajectory execution manager
        // only sends us the width of one finger. Multiply by 2 to get the intended width.
        double width = this->finger1_.getPosition() + this->finger2_.getPosition();
        {
          std::lock_guard<std::mutex> lock(this->mutex_);
          // Don't use goal as desired width, because we have might have to go beyond until contact
          this->width_desired_ = goal->command.position * 2.0 < width ? 0 : kMaxFingerWidth;
          this->speed_desired_ = this->speed_default_;
          this->force_desired_ = goal->command.max_effort;
          this->tolerance_.inner = this->tolerance_gripper_action_;
          this->tolerance_.outer = this->tolerance_gripper_action_;
          this->state_ = State::GRASPING;
        }

        this->waitUntil(State::HOLDING);

        double width_d = goal->command.position * 2.0;
        width = this->finger1_.getPosition() + this->finger2_.getPosition();  // recalculate
        bool inside_tolerance = width_d - this->tolerance_gripper_action_ < width and
                                width < width_d + this->tolerance_gripper_action_;
        control_msgs::GripperCommandResult result;
        result.position = width;
        result.effort = 0;
        result.stalled = static_cast<decltype(result.stalled)>(false);
        result.reached_goal = static_cast<decltype(result.reached_goal)>(inside_tolerance);
        double speed = this->finger1_.getVelocity() + this->finger2_.getVelocity();
        if (not inside_tolerance) {
          std::lock_guard<std::mutex> lock(this->mutex_);
          this->state_ = State::IDLE;
        }
        action_gc_->setSucceeded(result);
      },
      false);
  this->action_gc_->registerPreemptCallback([&]() {
    ROS_INFO_STREAM_NAMED("FrankaGripperSim", "Gripper Command Action cancelled");
    std::lock_guard<std::mutex> lock(this->mutex_);
    this->state_ = State::IDLE;
  });
  this->action_gc_->start();

  return true;
}

void FrankaGripperSim::starting(const ros::Time& /*unused*/) {
  this->pid1_.reset();
  this->pid2_.reset();
}

void FrankaGripperSim::update(const ros::Time& now, const ros::Duration& period) {
  if (rate_trigger_()) {
    std::lock_guard<std::mutex> lock(this->mutex_);
    pub_.msg_.header.stamp = now;
    pub_.msg_.position = {this->finger1_.getPosition(), this->finger2_.getPosition()};
    pub_.msg_.velocity = {this->finger1_.getVelocity(), this->finger2_.getVelocity()};
    pub_.msg_.effort = {this->finger1_.getEffort(), this->finger2_.getEffort()};
    pub_.unlockAndPublish();
  }

  // Read state threadsafe
  double width = this->finger1_.getPosition() + this->finger2_.getPosition();
  this->mutex_.lock();
  State state = this->state_;
  auto tolerance = this->tolerance_;
  double w_d = this->width_desired_;
  double dw_d = this->speed_desired_ * std::copysign(1.0, w_d - width);
  double f_d = -this->force_desired_ / 2.0;
  this->mutex_.unlock();

  if (state == State::IDLE) {
    // Track position of other finger to simulate mimicked joints + high damping
    control(this->finger1_, this->pid1_, this->finger2_.getPosition(), 0, 0, period);
    control(this->finger2_, this->pid2_, this->finger1_.getPosition(), 0, 0, period);
    return;
  }

  if (state != State::HOLDING) {
    // Only in case when we hold we want to add the desired force, in any other state don't add
    // anything extra to the command
    f_d = 0;
  }

  // Compute control signal and send to joints
  double w1_d = this->finger1_.getPosition() + 0.5 * dw_d * period.toSec();
  double w2_d = this->finger2_.getPosition() + 0.5 * dw_d * period.toSec();

  control(this->finger1_, this->pid1_, w1_d, 0.5 * dw_d, f_d, period);
  control(this->finger2_, this->pid2_, w2_d, 0.5 * dw_d, f_d, period);

  if (w_d - tolerance.inner < width and width < w_d + tolerance.outer) {
    // Goal reached, update statemachine
    std::lock_guard<std::mutex> lock(this->mutex_);
    if (state == State::MOVING) {
      // Done with move motion, switch to idle again
      this->speed_desired_ = 0;
      this->force_desired_ = 0;
      this->state_ = State::IDLE;
    }

    if (state == State::HOMING) {
      if (this->width_desired_ == 0) {
        // Finger now open, first part of homing done, switch direction
        this->width_desired_ = kMaxFingerWidth;
      } else {
        // Finger now closed again, homing finished
        this->state_ = State::IDLE;
      }
    }
  }

  if (state == State::GRASPING) {
    // Since the velocity signal is noisy it can easily happen that one sample is below the
    // threshold To avoid abortion because of noise, we have to read at least N consecutive number
    // of samples before interpreting something was grasped (or not)
    static int speed_threshold_counter = 0;
    double speed = this->finger1_.getVelocity() + this->finger2_.getVelocity();
    if (std::abs(speed) <= this->speed_threshold_) {
      speed_threshold_counter++;
    } else {
      speed_threshold_counter = 0;
    }

    if (speed_threshold_counter >= this->speed_samples_) {
      std::lock_guard<std::mutex> lock(this->mutex_);
      // Done with grasp motion, switch to holding, i.e. keep position & force
      this->speed_desired_ = 0;
      this->state_ = State::HOLDING;
      speed_threshold_counter = 0;
    }
  }
}

double FrankaGripperSim::control(hardware_interface::JointHandle& joint,
                                 control_toolbox::Pid& pid,
                                 double q_d,
                                 double dq_d,
                                 double f_d,
                                 const ros::Duration& period) {
  double error = q_d - joint.getPosition();
  double derror = dq_d - joint.getVelocity();
  double command = pid.computeCommand(error, derror, period);
  command += f_d;
  joint.setCommand(command);
  return command;
}

void FrankaGripperSim::interrupt(const std::string& message, const State& except) {
  if (except != State::MOVING and this->action_move_ != nullptr and
      this->action_move_->isActive()) {
    franka_gripper::MoveResult result;
    result.success = static_cast<decltype(result.success)>(false);
    result.error = message;
    this->action_move_->setAborted(result, result.error);
  }
  if (except != State::GRASPING and this->action_grasp_ != nullptr and
      this->action_grasp_->isActive()) {
    franka_gripper::GraspResult result;
    result.success = static_cast<decltype(result.success)>(false);
    result.error = message;
    this->action_grasp_->setAborted(result, result.error);
  }
  if (except != State::HOMING and this->action_homing_ != nullptr and
      this->action_homing_->isActive()) {
    franka_gripper::HomingResult result;
    result.success = static_cast<decltype(result.success)>(false);
    result.error = message;
    this->action_homing_->setAborted(result, result.error);
  }
}

void FrankaGripperSim::waitUntil(const State& state) {
  ros::Rate rate(30);
  while (true) {
    {
      std::lock_guard<std::mutex> lock(this->mutex_);
      if (this->state_ == state) {
        return;
      }
    }
    rate.sleep();
  }
}

}  // namespace franka_gazebo

PLUGINLIB_EXPORT_CLASS(franka_gazebo::FrankaGripperSim, controller_interface::ControllerBase);
