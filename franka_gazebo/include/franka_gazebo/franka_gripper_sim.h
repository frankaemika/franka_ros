#pragma once

#include <functional>
#include <mutex>

#include <actionlib/server/simple_action_server.h>
#include <control_msgs/GripperCommandAction.h>
#include <control_toolbox/pid.h>
#include <controller_interface/controller.h>
#include <franka_gripper/GraspAction.h>
#include <franka_gripper/GraspEpsilon.h>
#include <franka_gripper/HomingAction.h>
#include <franka_gripper/MoveAction.h>
#include <franka_gripper/StopAction.h>
#include <franka_hw/trigger_rate.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/time.h>
#include <sensor_msgs/JointState.h>

namespace franka_gazebo {

const double kMaxFingerWidth = 0.08;

/// When width between fingers is below this, the move action succeeds [m]
const double kDefaultMoveWidthTolerance = 0.001;

/// When width between fingers is below this, the gripper action succeeds [m]
const double kDefaultGripperActionWidthTolerance = 0.005;

/// How fast shall the gripper execute gripper command actions? [m/s]
const double kDefaultGripperActionSpeed = 0.1;

/// Below which speed the target width should be checked to abort or succeed the grasp action [m/s]
const double kGraspRestingThreshold = 0.003;

/// How many times the speed has to drop below resting threshold before the grasping will be checked
const int kGraspConsecutiveSamples = 10;

/**
 * Simulate the franka_gripper_node.
 *
 * Internally this is done via ROS control. This controller assumes there are two finger joints
 * in the URDF which can be effort (force) controlled. It simulates the behavior of the real
 * franka_gripper by offering the same actions:
 *
 * - homing:  Execute a homing motion, i.e open and close the gripper fully. This is only
 *            a mock implementation for the sake of offering the complete action interface
 *            similar to franka_gripper
 * - move:    Move the gripper with a desired velocity to a certain width.
 * - grasp:   Close the gripper until it stops because of a contact. If then the gripper width
 * is within a user specified range a certain force is applied
 * - stop:    Stop any previous motion, or the excertion of forces on currently grasped objects
 * - gripper_action: A standard gripper action recognized by MoveIt!
 *
 * NOTE: The `grasp` action has a bug, that it will not succeed nor abort if the target width
 *       lets the fingers open. This is because of missing the joint limits interface which
 *       lets the finger oscillate at their limits.
 */
class FrankaGripperSim
    : public controller_interface::Controller<hardware_interface::EffortJointInterface> {
 public:
  enum State {
    IDLE,      ///< Gripper is not actively controlled, but tracks the other finger to simulate a
               ///< mimicked joint
    HOLDING,   ///< Gripper is holding position and tracking zero velocity while mainting a desired
               ///< force
    MOVING,    ///< Gripper is tracking a desired position and velocity
    GRASPING,  ///< Gripper is tracking a desired position and velocity. On contact it switches to
               ///< `HOLDING` if inside the epsilon of the desired grasping width otherwise back to
               ///< `IDLE`
  };

  struct Config {
    double width_desired;  ///< Desired width between both fingers [m]
    double speed_desired;  ///< Desired magnitude of the speed with which fingers should move [m/s]
    double force_desired;  ///< Desired force with which to grasp objects, if grasps succeed [N]
    franka_gripper::GraspEpsilon tolerance;  ///< Tolerance range to check if grasping succeeds
  };

  bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& nh) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time& now, const ros::Duration& period) override;

 private:
  State state_ = State::IDLE;
  Config config_;

  franka_hw::TriggerRate rate_trigger_{30.0};
  control_toolbox::Pid pid1_;
  control_toolbox::Pid pid2_;
  realtime_tools::RealtimePublisher<sensor_msgs::JointState> pub_;
  hardware_interface::JointHandle finger1_;
  hardware_interface::JointHandle finger2_;

  std::mutex mutex_;

  // Configurable by parameters
  int speed_samples_;
  double speed_threshold_;
  double speed_default_;
  double tolerance_move_;            ///< [m] inner + outer position tolerances used during grasp
  double tolerance_gripper_action_;  ///< [m] inner + outer position tolerances used during gripper
                                     ///< action

  std::unique_ptr<actionlib::SimpleActionServer<franka_gripper::StopAction>> action_stop_;
  std::unique_ptr<actionlib::SimpleActionServer<franka_gripper::HomingAction>> action_homing_;
  std::unique_ptr<actionlib::SimpleActionServer<franka_gripper::MoveAction>> action_move_;
  std::unique_ptr<actionlib::SimpleActionServer<franka_gripper::GraspAction>> action_grasp_;
  std::unique_ptr<actionlib::SimpleActionServer<control_msgs::GripperCommandAction>> action_gc_;

  void setState(const State&& state);
  void setConfig(const Config&& config);
  void transition(const State&& state, const Config&& config);

  void control(hardware_interface::JointHandle& joint,
               control_toolbox::Pid&,
               double q_d,
               double dq_d,
               double f_d,
               const ros::Duration& period);

  /**
   * Interrupt any running action server unless the gripper is currently in a specific state
   *
   * @param[in] message The message to send via the result of all running actions
   * @param[in] except   If the gripper is currently in this state, don't interrupt any actions
   */
  void interrupt(const std::string& message, const State& except);

  void waitUntilStateChange();

  void onStopGoal(const franka_gripper::StopGoalConstPtr& goal);
  void onHomingGoal(const franka_gripper::HomingGoalConstPtr& goal);
  void onMoveGoal(const franka_gripper::MoveGoalConstPtr& goal);
  void onGraspGoal(const franka_gripper::GraspGoalConstPtr& goal);
  void onGripperActionGoal(const control_msgs::GripperCommandGoalConstPtr& goal);

  /**
   * libfranka-like method to grasp an object with the gripper
   * @param[in] width Size of the object to grasp. [m]
   * @param[in] speed Closing speed. [m/s]
   * @param[in] force Grasping force. [N]
   * @param[in] epsilon Maximum tolerated deviation between the commanded width and the desired
   * width
   * @return True if the object could be grasped, false otherwise
   */
  bool grasp(double width, double speed, double force, const franka_gripper::GraspEpsilon& epsilon);

  /**
   * libfranka-like method to move the gripper to a certain position
   * @param[in] width Intended opening width. [m]
   * @param[in] speed Closing speed. [m/s]
   * @return True if the command was successful, false otherwise.
   */
  bool move(double width, double speed);
};
}  // namespace franka_gazebo
