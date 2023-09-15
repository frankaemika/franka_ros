#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <franka_gripper/GraspAction.h>
#include <franka_gripper/MoveAction.h>
#include <gtest/gtest.h>
#include <ros/ros.h>

#include "gripper_sim_test_setup.h"

static const double kAllowedPositionError = 5e-3;
static const double kAllowedForceError = 3;
static const double kStoneWidth = 0.032;

class GripperGraspFixtureTest
    : public GripperSimTestSetup,
      public testing::WithParamInterface<
          std::tuple<std::tuple<double, double, double, double, double, double>, double>> {
 protected:
  void SetUp() override {
    GripperSimTestSetup::SetUp();
    std::tie(desired_width, desired_velocity, desired_force, desired_epsilon_inner,
             desired_epsilon_outer, allowed_absolute_duration_error) = std::get<0>(GetParam());
    desired_sleep = std::get<1>(GetParam());
  }

  double desired_width{0};
  double desired_velocity{0};
  double desired_force{0};
  double desired_epsilon_inner{0};
  double desired_epsilon_outer{0};
  double allowed_absolute_duration_error{0};
  double desired_sleep{0};
};

class GripperFailGraspFixtureTest : public GripperGraspFixtureTest {};

TEST_F(GripperSimTestSetup, FailMove) {  // NOLINT(cert-err58-cpp)
  double desired_width = 0;
  double desired_velocity = 0.1;

  auto move_goal = franka_gripper::MoveGoal();
  move_goal.width = desired_width;
  move_goal.speed = desired_velocity;
  move_client->sendGoal(move_goal);

  bool finished_before_timeout = move_client->waitForResult(ros::Duration(15.0));
  EXPECT_TRUE(finished_before_timeout);
  EXPECT_EQ(move_client->getState(), actionlib::SimpleClientGoalState::SUCCEEDED);
  EXPECT_FALSE(move_client->getResult()->success);

  move_goal.width = 0.08;
  move_client->sendGoal(move_goal);
  finished_before_timeout = move_client->waitForResult(ros::Duration(15.0));
  EXPECT_TRUE(finished_before_timeout);
  EXPECT_EQ(move_client->getState(), actionlib::SimpleClientGoalState::SUCCEEDED);
  EXPECT_TRUE(move_client->getResult()->success);
}

TEST_P(GripperGraspFixtureTest, CanGrasp) {  // NOLINT(cert-err58-cpp)
  double start_width = 0.08;
  auto move_goal = franka_gripper::MoveGoal();
  move_goal.width = start_width;
  move_goal.speed = 0.1;
  this->move_client->sendGoal(move_goal);
  bool finished_before_timeout = move_client->waitForResult(ros::Duration(10.0));
  resetStone();
  EXPECT_TRUE(finished_before_timeout);
  ros::Duration(desired_sleep).sleep();
  double expected_duration = (start_width - kStoneWidth) / desired_velocity;
  auto grasp_goal = franka_gripper::GraspGoal();
  grasp_goal.width = desired_width;
  grasp_goal.speed = desired_velocity;
  grasp_goal.force = desired_force;
  grasp_goal.epsilon.inner = desired_epsilon_inner;
  grasp_goal.epsilon.outer = desired_epsilon_outer;
  auto start_time = ros::Time::now();
  this->grasp_client->sendGoal(grasp_goal);
  finished_before_timeout = this->grasp_client->waitForResult(ros::Duration(10.0));
  auto stop_time = ros::Time::now();
  double duration = (stop_time - start_time).toSec();
  EXPECT_TRUE(finished_before_timeout);
  EXPECT_NEAR(duration, expected_duration, allowed_absolute_duration_error);
  EXPECT_EQ(grasp_client->getState(), actionlib::SimpleClientGoalState::SUCCEEDED);
  EXPECT_TRUE(grasp_client->getResult()->success);
  for (int i = 0; i < 40; i++) {
    ros::Duration(0.1).sleep();
    updateFingerState();
    EXPECT_NEAR(finger_1_pos * 2, kStoneWidth, kAllowedPositionError);
    EXPECT_NEAR(finger_2_pos, finger_1_pos, kAllowedPositionError);
    if (ROS_VERSION_MINIMUM(1, 15, 0)) {  // only test forces in noetic and later versions
      double expected_force = desired_force / 2.0;
      EXPECT_NEAR(finger_1_force, expected_force, kAllowedForceError);
      EXPECT_NEAR(finger_2_force, expected_force, kAllowedForceError);
    }
  }
}

TEST_P(GripperFailGraspFixtureTest, CanFailGrasp) {  // NOLINT(cert-err58-cpp)
  double start_width = 0.08;
  auto move_goal = franka_gripper::MoveGoal();
  move_goal.width = start_width;
  move_goal.speed = 0.1;
  this->move_client->sendGoal(move_goal);
  bool finished_before_timeout = move_client->waitForResult(ros::Duration(10.0));
  EXPECT_TRUE(finished_before_timeout);
  resetStone();
  ros::Duration(desired_sleep).sleep();
  double expected_duration = (start_width - kStoneWidth) / desired_velocity;
  auto grasp_goal = franka_gripper::GraspGoal();
  grasp_goal.width = desired_width;
  grasp_goal.speed = desired_velocity;
  grasp_goal.force = desired_force;
  grasp_goal.epsilon.inner = desired_epsilon_inner;
  grasp_goal.epsilon.outer = desired_epsilon_outer;
  auto start_time = ros::Time::now();
  this->grasp_client->sendGoal(grasp_goal);
  finished_before_timeout = this->grasp_client->waitForResult(ros::Duration(10.0));
  auto stop_time = ros::Time::now();
  double duration = (stop_time - start_time).toSec();
  updateFingerState();
  EXPECT_TRUE(finished_before_timeout);
  EXPECT_NEAR(finger_1_pos * 2, kStoneWidth, kAllowedPositionError);
  EXPECT_NEAR(finger_2_pos * 2, kStoneWidth, kAllowedPositionError);
  EXPECT_NEAR(duration, expected_duration, allowed_absolute_duration_error);
  EXPECT_EQ(grasp_client->getState(), actionlib::SimpleClientGoalState::SUCCEEDED);
  EXPECT_FALSE(grasp_client->getResult()->success);
}

INSTANTIATE_TEST_CASE_P(
    DISABLED_GripperFailGraspFixtureTest,  // NOLINT(cert-err58-cpp)
    GripperFailGraspFixtureTest,
    ::testing::Combine(::testing::Values(std::make_tuple(0.04, 0.1, 0., 0.005, 0.005, 0.1),
                                         std::make_tuple(0.02, 0.1, 2., 0.01, 0.01, 0.1),
                                         std::make_tuple(0.03, 0.1, 2, 0.001, 0.001, 0.1)),
                       ::testing::Values(0, 0.1)));

INSTANTIATE_TEST_CASE_P(
    DISABLED_GripperGraspFixtureTest,  // NOLINT(cert-err58-cpp)
    GripperGraspFixtureTest,
    ::testing::Combine(::testing::Values(std::make_tuple(0.032, 0.1, 100., 0.01, 0.01, 0.1),
                                         std::make_tuple(0.03, 0.1, 30., 0.005, 0.005, 0.1),
                                         std::make_tuple(0.0325, 0.01, 30., 0.003, 0.003, 0.6),
                                         std::make_tuple(0.034, 0.01, 30., 0.005, 0.005, 0.6)),
                       ::testing::Values(0, 0.1)));

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "franka_gripper_sim_test_with_object");
  return RUN_ALL_TESTS();
}
