#include <actionlib/client/simple_action_client.h>
#include <franka/robot_state.h>
#include <franka_gazebo/franka_gripper_sim.h>
#include <franka_gazebo/joint.h>
#include <franka_gazebo/statemachine.h>
#include <franka_gripper/MoveAction.h>
#include <franka_msgs/ErrorRecoveryAction.h>
#include <franka_msgs/FrankaState.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_srvs/SetBool.h>
#include <array>
#include <boost_sml/sml.hpp>
#include <map>
#include <memory>
#include <string>

using namespace franka_gazebo;
using franka::RobotMode;
namespace sml = boost::sml;

// NOTE: Keep a global node handle in memory in order for all tests to be executed sequentially
// If each test or fixture declares its own node handle only the first test will be executed
// correctly
std::unique_ptr<ros::NodeHandle> nh;

TEST(
#if ROS_VERSION_MINIMUM(1, 15, 13)
    FrankaHWSim, /* noetic & newer */
#else
    DISABLED_FrankaHWSim, /* melodic */
#endif
    DISABLED_franka_hw_sim_compensates_gravity_on_F_ext) {  // TODO(goll_th) enable again when not
                                                            // flaky anymore
  ros::NodeHandle n;

  for (int i = 0; i < 50; i++) {
    auto msg = ros::topic::waitForMessage<geometry_msgs::WrenchStamped>(
        "/franka_state_controller/F_ext", n);

    auto now = msg->header.stamp;

    EXPECT_LE(std::abs(msg->wrench.force.x), 0.5) << "During time: " << now;
    EXPECT_LE(std::abs(msg->wrench.force.y), 0.5) << "During time: " << now;
    EXPECT_LE(std::abs(msg->wrench.force.z), 0.5) << "During time: " << now;
    EXPECT_LE(std::abs(msg->wrench.torque.x), 0.25) << "During time: " << now;
    EXPECT_LE(std::abs(msg->wrench.torque.y), 0.25) << "During time: " << now;
    EXPECT_LE(std::abs(msg->wrench.torque.z), 0.25) << "During time: " << now;
  }
}

class StateMachineFixture : public ::testing::Test {
 protected:
  franka::RobotState state;
  std::map<std::string, std::shared_ptr<Joint>> joints;
  std::unique_ptr<sml::sm<StateMachine, sml::testing>> sm;

  virtual void SetUp() {
    sm = std::make_unique<sml::sm<StateMachine, sml::testing>>(state, joints);
    for (int i = 1; i <= 7; i++) {
      joints.emplace(armJoint(i), std::make_unique<Joint>());
    }
    joints.emplace("panda_finger_joint1", std::make_unique<Joint>());
    joints.emplace("panda_finger_joint2", std::make_unique<Joint>());
  }

  std::string armJoint(int i) { return "panda_joint" + std::to_string(i); }
};

TEST_F(StateMachineFixture, statemachine_initializes_in_Idle) {
  ASSERT_TRUE(sm->is(sml::state<Idle>));
}

TEST_F(StateMachineFixture, statemachine_switches_from_Idle_to_Move_on_error_recovery_event) {
  sm->set_current_states(sml::state<Idle>);
  sm->process_event(ErrorRecovery());
  EXPECT_TRUE(sm->is(sml::state<Move>));
}

TEST_F(StateMachineFixture,
       statemachine_switches_from_Idle_to_UserStopped_if_user_stop_is_pressed) {
  sm->set_current_states(sml::state<Idle>);
  sm->process_event(UserStop{true});
  EXPECT_TRUE(sm->is(sml::state<UserStopped>));
}

TEST_F(StateMachineFixture, statemachine_switches_from_Move_to_UserStopped_on_user_stop_event) {
  sm->set_current_states(sml::state<Move>);
  sm->process_event(UserStop{true});
  EXPECT_TRUE(sm->is(sml::state<UserStopped>));
}

TEST_F(StateMachineFixture, statemachine_stays_in_Move_on_user_stop_event) {
  sm->set_current_states(sml::state<Move>);
  sm->process_event(UserStop{false});
  EXPECT_TRUE(sm->is(sml::state<Move>));
}

TEST_F(StateMachineFixture, statemachine_stays_in_Move_on_error_recovery_event) {
  sm->set_current_states(sml::state<Move>);
  sm->process_event(ErrorRecovery());
  EXPECT_TRUE(sm->is(sml::state<Move>));
}

TEST_F(StateMachineFixture, statemachine_stays_in_UserStopped_if_user_stop_still_pressed) {
  sm->set_current_states(sml::state<UserStopped>);
  sm->process_event(UserStop{true});
  EXPECT_TRUE(sm->is(sml::state<UserStopped>));
}

TEST_F(StateMachineFixture,
       statemachine_switches_from_UserStopped_to_Idle_if_user_stop_is_released) {
  sm->set_current_states(sml::state<UserStopped>);
  sm->process_event(UserStop{false});
  EXPECT_TRUE(sm->is(sml::state<Idle>));
}

TEST_F(StateMachineFixture, statemachine_switches_from_Idle_to_Move_if_joints_are_controlled) {
  // Arange
  sm->set_current_states(sml::state<Idle>);

  // Act
  for (int i = 1; i <= 7; i++) {
    joints.at(armJoint(i))->control_method = ControlMethod::POSITION;
  }
  sm->process_event(SwitchControl());

  // Assert
  EXPECT_TRUE(sm->is(sml::state<Move>));
}

TEST_F(StateMachineFixture, statemachine_switches_from_Move_to_Idle_if_joints_are_uncontrolled) {
  // Arange
  sm->set_current_states(sml::state<Move>);

  // Act
  for (int i = 1; i <= 7; i++) {
    joints.at(armJoint(i))->control_method = boost::none;
  }
  sm->process_event(SwitchControl());

  // Assert
  EXPECT_TRUE(sm->is(sml::state<Idle>));
}

TEST_F(StateMachineFixture, statemachine_stays_in_Idle_if_only_finger_joints_are_controlled) {
  // Arange
  sm->set_current_states(sml::state<Idle>);

  // Act
  joints.at("panda_finger_joint1")->control_method = ControlMethod::POSITION;
  joints.at("panda_finger_joint2")->control_method = ControlMethod::POSITION;
  sm->process_event(SwitchControl());

  // Assert
  EXPECT_TRUE(sm->is(sml::state<Idle>));
}

TEST_F(StateMachineFixture, statemachine_stays_in_Move_if_only_finger_joints_are_uncontrolled) {
  // Arange
  sm->set_current_states(sml::state<Move>);
  for (int i = 1; i <= 7; i++) {
    joints.at(armJoint(i))->control_method = ControlMethod::VELOCITY;
  }

  // Act
  joints.at("panda_finger_joint1")->control_method = boost::none;
  joints.at("panda_finger_joint2")->control_method = boost::none;
  sm->process_event(SwitchControl());

  // Assert
  EXPECT_TRUE(sm->is(sml::state<Move>));
}

TEST_F(StateMachineFixture, statemachine_updates_robot_mode_when_switching_from_Idle_to_Move) {
  // Arange
  sm->set_current_states(sml::state<Idle>);
  state.robot_mode = franka::RobotMode::kIdle;

  // Act
  for (int i = 1; i <= 7; i++) {
    joints.at(armJoint(i))->control_method = ControlMethod::VELOCITY;
  }
  sm->process_event(SwitchControl());

  // Assert
  ASSERT_EQ(state.robot_mode, franka::RobotMode::kMove);
}

TEST_F(StateMachineFixture, statemachine_updates_robot_mode_when_switching_from_Move_to_Idle) {
  // Arange
  sm->set_current_states(sml::state<Move>);
  state.robot_mode = franka::RobotMode::kMove;

  // Act
  for (int i = 1; i <= 7; i++) {
    joints.at(armJoint(i))->control_method = boost::none;
  }
  sm->process_event(SwitchControl());

  // Assert
  ASSERT_EQ(state.robot_mode, franka::RobotMode::kIdle);
}

TEST_F(StateMachineFixture,
       statemachine_updates_robot_mode_when_switching_from_Idle_to_UserStopped) {
  sm->set_current_states(sml::state<Idle>);
  state.robot_mode = franka::RobotMode::kIdle;

  sm->process_event(UserStop{true});
  ASSERT_EQ(state.robot_mode, franka::RobotMode::kUserStopped);
}

TEST_F(StateMachineFixture,
       statemachine_updates_robot_mode_when_switching_from_Move_to_UserStopped) {
  sm->set_current_states(sml::state<Move>);
  state.robot_mode = franka::RobotMode::kMove;

  sm->process_event(UserStop{true});
  ASSERT_EQ(state.robot_mode, franka::RobotMode::kUserStopped);
}

TEST_F(StateMachineFixture,
       statemachine_updates_robot_mode_when_switching_from_UserStopped_to_Idle) {
  sm->set_current_states(sml::state<UserStopped>);
  state.robot_mode = franka::RobotMode::kUserStopped;

  sm->process_event(UserStop{false});
  ASSERT_EQ(state.robot_mode, franka::RobotMode::kIdle);
}

TEST_F(StateMachineFixture,
       statemachine_resets_desired_control_signals_when_switching_from_Idle_to_UserStopped) {
  // Arange
  sm->set_current_states(sml::state<Idle>);
  for (int i = 1; i <= 7; i++) {
    auto& joint = joints.at(armJoint(i));
    joint->position = 100 + i;
    joint->desired_position = 42 + i;
    joint->desired_velocity = 84 + i;
    joint->stop_position = 0;
  }

  // Act
  sm->process_event(UserStop{true});

  // Assert
  for (int i = 1; i <= 7; i++) {
    auto& joint = joints.at(armJoint(i));
    ASSERT_DOUBLE_EQ(joint->position, 100 + i) << "Joint position changed but expected it didn't";
    ASSERT_DOUBLE_EQ(joint->stop_position, joint->position);
    ASSERT_DOUBLE_EQ(joint->desired_position, joint->position);
    ASSERT_DOUBLE_EQ(joint->desired_velocity, 0);
  }

  std::array<double, 7> zeros = {0};
  ASSERT_EQ(state.q_d, state.q);
  ASSERT_EQ(state.dq_d, zeros);
  ASSERT_EQ(state.ddq_d, zeros);
}

TEST_F(StateMachineFixture,
       statemachine_resets_desired_control_signals_when_switching_from_Move_to_UserStopped) {
  // Arange
  sm->set_current_states(sml::state<Move>);
  for (int i = 1; i <= 7; i++) {
    auto& joint = joints.at(armJoint(i));
    joint->position = 100 + i;
    joint->desired_position = 42 + i;
    joint->desired_velocity = 84 + i;
    joint->stop_position = 0;
  }

  // Act
  sm->process_event(UserStop{true});

  // Assert
  for (int i = 1; i <= 7; i++) {
    auto& joint = joints.at(armJoint(i));
    ASSERT_DOUBLE_EQ(joint->position, 100 + i) << "Joint position changed but expected it didn't";
    ASSERT_DOUBLE_EQ(joint->stop_position, joint->position);
    ASSERT_DOUBLE_EQ(joint->desired_position, joint->position);
    ASSERT_DOUBLE_EQ(joint->desired_velocity, 0);
  }

  std::array<double, 7> zeros = {0};
  ASSERT_EQ(state.q_d, state.q);
  ASSERT_EQ(state.dq_d, zeros);
  ASSERT_EQ(state.ddq_d, zeros);
}

struct FrankaHWSimFixture : public ::testing::Test {
 private:
  double start_;

 public:
  ros::ServiceClient user_stop;
  std::unique_ptr<actionlib::SimpleActionClient<franka_gripper::MoveAction>> gripper;
  std::unique_ptr<actionlib::SimpleActionClient<franka_msgs::ErrorRecoveryAction>> error_recovery;

  virtual void SetUp() {
    start_ = ros::Time::now().toSec();
    user_stop = nh->serviceClient<std_srvs::SetBool>("/franka_control/set_user_stop");
    gripper = std::make_unique<actionlib::SimpleActionClient<franka_gripper::MoveAction>>(
        "/franka_gripper/move");
    error_recovery =
        std::make_unique<actionlib::SimpleActionClient<franka_msgs::ErrorRecoveryAction>>(
            "/franka_control/error_recovery");
  }

  double now() { return ros::Time::now().toSec() - start_; }

  void openGripper() {
    franka_gripper::MoveGoal goal;
    goal.speed = 0.1;
    goal.width = franka_gazebo::kMaxFingerWidth;
    gripper->sendGoalAndWait(goal, ros::Duration(5));
    auto result = gripper->getResult();
    ASSERT_TRUE(result->success) << "Failed to open the gripper, error: " << result->error;
  }
  void closeGripper() {
    franka_gripper::MoveGoal goal;
    goal.speed = 0.1;
    goal.width = 0.0;
    gripper->sendGoalAndWait(goal, ros::Duration(5));
    auto result = gripper->getResult();
    ASSERT_TRUE(result->success) << "Failed to close the gripper, error: " << result->error;
  }

  void errorRecovery() {
    error_recovery->sendGoalAndWait(franka_msgs::ErrorRecoveryGoal(), ros::Duration(5));
  }

  std_srvs::SetBool::Response setUserStop(bool pressed) {
    std_srvs::SetBool service;
    service.request.data = pressed;
    user_stop.call(service);
    return service.response;
  }

  virtual void TearDown() {
    closeGripper();
    setUserStop(false);
    errorRecovery();
  }
};

TEST_F(FrankaHWSimFixture, pressing_user_stop_stops_the_arm_joints) {
  // Test procedure:
  // Command a circular motion to the impedance controller to get the joints moving
  // After some time (user_stop_time) press the user stop, let the joints stop as fast as possible
  // After again some time (assert_start_time) measure the velocities of all joints
  // After again some time (assert_end_time) verify if the average of the velocities is below some
  // threshold. Have to filter because velocity signal is noisy

  double radius = 0.10;
  double frequency = 0.5;
  double user_stop_time = 3;
  double assert_start_time = 3.5;
  double assert_end_time = 4.5;
  double stop_speed_tolerance = 15e-3;  // [rad/s]

  ros::Duration timeout(5);

  geometry_msgs::PoseStamped target;
  target.pose.position.z = 0.6;
  target.pose.orientation.x = 1;
  target.header.frame_id = "panda_link0";
  size_t samples = 0;
  std::map<std::string, double> velocities;
  auto pub = nh->advertise<geometry_msgs::PoseStamped>(
      "/cartesian_impedance_example_controller/equilibrium_pose", 1);
  ros::Rate rate(30);

  bool user_stop_pressed = false;
  while (ros::ok()) {
    rate.sleep();
    double t = now();

    target.pose.position.x = radius * std::sin(2 * M_PI * frequency * t) + 0.3;
    target.pose.position.y = radius * std::cos(2 * M_PI * frequency * t);
    pub.publish(target);

    if (t < user_stop_time) {
      continue;
    }
    // After some time, we press the user stop
    if (not user_stop_pressed) {
      setUserStop(true);
      user_stop_pressed = true;
    }
    if (t < assert_start_time) {
      continue;
    }

    auto state = ros::topic::waitForMessage<franka_msgs::FrankaState>(
        "/franka_state_controller/franka_states", *nh, timeout);
    ASSERT_TRUE(state != nullptr)
        << "No message on /franka_state_controller/franka_states received within "
        << timeout.toSec() << "s";
    EXPECT_EQ(static_cast<RobotMode>(state->robot_mode), RobotMode::kUserStopped);

    // A little while later we check if the joints are still moving
    auto msg = ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states", *nh, timeout);
    ASSERT_TRUE(msg != nullptr) << "No message on /joint_states received within " << timeout.toSec()
                                << "s";
    for (int i = 0; i < msg->name.size(); i++) {
      auto name = msg->name.at(i);
      if (contains(name, "finger_joint")) {
        continue;
      }
      velocities.emplace(name, 0);  // insert if not yet exist
      velocities.at(name) += msg->velocity.at(i);
    }
    samples++;

    if (t < assert_end_time) {
      continue;
    }

    for (auto pair : velocities) {
      auto name = pair.first;
      auto velocity = pair.second / samples;
      EXPECT_NEAR(velocity, 0, stop_speed_tolerance) << "Joint " << name << " is still moving!";
    }

    break;  // finish test
  }
}

TEST_F(FrankaHWSimFixture, pressing_user_stop_lets_finger_joints_still_move) {
  // Test Procedure:
  // Arange by closing the gripper after (close_gripper_time)
  // After some time (user_stop_time) the user stop is pressed
  // Then after some time (open_gripper_time) make a move action call to open the fingers
  // When the action is finished, check if the finger positions are actually where we commanded them

  double user_stop_time = 1;
  double open_gripper_time = 1.5;
  double position_tolerance = 5e-3;  // [m]

  bool pressed_user_stop = false;
  ros::Rate rate(30);
  ros::Duration timeout(5);

  while (ros::ok()) {
    rate.sleep();

    double t = now();

    if (t < user_stop_time) {
      continue;
    }

    if (not pressed_user_stop) {
      pressed_user_stop = true;
      setUserStop(true);
    }

    if (t < open_gripper_time) {
      continue;
    }
    openGripper();

    auto msg = ros::topic::waitForMessage<sensor_msgs::JointState>("/franka_gripper/joint_states",
                                                                   *nh, timeout);
    ASSERT_TRUE(msg != nullptr) << "No message on /franka_gripper/joint_states received within "
                                << timeout.toSec() << "s";
    for (int i = 0; i < msg->name.size(); i++) {
      auto name = msg->name.at(i);
      if (not contains(name, "finger_joint")) {
        continue;
      }
      EXPECT_NEAR(msg->position.at(i), franka_gazebo::kMaxFingerWidth / 2., position_tolerance);
    }

    break;  // finish test
  }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "franka_hw_sim_test");
  nh = std::make_unique<ros::NodeHandle>();
  return RUN_ALL_TESTS();
}
