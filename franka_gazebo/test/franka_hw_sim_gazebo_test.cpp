#include <franka/robot_state.h>
#include <franka_gazebo/joint.h>
#include <franka_gazebo/statemachine.h>
#include <geometry_msgs/WrenchStamped.h>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <array>
#include <boost_sml/sml.hpp>
#include <map>
#include <memory>
#include <string>

using namespace franka_gazebo;
namespace sml = boost::sml;
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

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "franka_hw_sim_test");
  return RUN_ALL_TESTS();
}
