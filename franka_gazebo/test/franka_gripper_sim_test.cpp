#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <franka_gripper/GraspAction.h>
#include <franka_gripper/HomingAction.h>
#include <franka_gripper/MoveAction.h>
#include <gtest/gtest.h>
#include <ros/ros.h>

#include "gripper_sim_test_setup.h"

static const double kAllowedPositionError = 5e-3;

class GripperMoveFixtureTest
    : public GripperSimTestSetup,
      public testing::WithParamInterface<std::tuple<double, double, double>> {
 protected:
  void SetUp() override {
    GripperSimTestSetup::SetUp();
    std::tie(desired_width, desired_velocity, allowed_absolute_duration_error) = GetParam();
  }

  double desired_width{0};
  double desired_velocity{0};
  double allowed_absolute_duration_error{0};
};

class GripperGraspFixtureTest
    : public GripperSimTestSetup,
      public testing::WithParamInterface<std::tuple<double, double, double, double>> {
 protected:
  void SetUp() override {
    GripperSimTestSetup::SetUp();
    std::tie(desired_width, desired_velocity, desired_force, allowed_absolute_duration_error) =
        GetParam();
  }

  double desired_width{0};
  double desired_velocity{0};
  double desired_force{0};
  double allowed_absolute_duration_error{0};
};

class GripperGraspZeroFixtureTest
    : public GripperSimTestSetup,
      public testing::WithParamInterface<
          std::tuple<std::tuple<double, double, double, double>, double>> {
 protected:
  void SetUp() override {
    GripperSimTestSetup::SetUp();
    std::tie(desired_width, desired_velocity, desired_force, allowed_absolute_duration_error) =
        std::get<0>(GetParam());
    desired_sleep = std::get<1>(GetParam());
  }

  double desired_width{0};
  double desired_velocity{0};
  double desired_force{0};
  double allowed_absolute_duration_error{0};
  double desired_sleep{0};
};

class GripperFailMoveFixtureTest : public GripperMoveFixtureTest {};

class GripperHomingFixtureTest : public GripperSimTestSetup,
                                 public testing::WithParamInterface<double> {
 protected:
  double desired_width{GetParam()};
};

TEST_P(GripperFailMoveFixtureTest, FailMove) {  // NOLINT(cert-err58-cpp)
  auto move_goal = franka_gripper::MoveGoal();
  move_goal.width = desired_width;
  move_goal.speed = desired_velocity;
  move_client->sendGoal(move_goal);

  bool finished_before_timeout = move_client->waitForResult(ros::Duration(15.0));
  EXPECT_TRUE(finished_before_timeout);
  EXPECT_EQ(move_client->getState(), actionlib::SimpleClientGoalState::ABORTED);
  EXPECT_FALSE(move_client->getResult()->success);
}

TEST_P(GripperMoveFixtureTest, CanPerformMove) {  // NOLINT(cert-err58-cpp)
  auto move_goal = franka_gripper::MoveGoal();
  double start_width = finger_1_pos * 2;
  double expected_duration = std::abs(start_width - desired_width) / desired_velocity;

  move_goal.width = desired_width;
  move_goal.speed = desired_velocity;
  auto start_time = ros::Time::now();
  move_client->sendGoal(move_goal);
  bool finished_before_timeout = move_client->waitForResult(ros::Duration(15.0));
  auto stop_time = ros::Time::now();
  double duration = (stop_time - start_time).toSec();
  updateFingerState();

  EXPECT_TRUE(finished_before_timeout);
  EXPECT_EQ(move_client->getState(), actionlib::SimpleClientGoalState::SUCCEEDED);
  EXPECT_TRUE(move_client->getResult()->success);
  EXPECT_NEAR(finger_1_pos * 2, desired_width, kAllowedPositionError);
  EXPECT_NEAR(finger_2_pos * 2, desired_width, kAllowedPositionError);
  EXPECT_NEAR(duration, expected_duration, allowed_absolute_duration_error);
}

TEST_P(GripperHomingFixtureTest, CanPerformHoming) {  // NOLINT(cert-err58-cpp)
  double desired_velocity = 0.01;

  auto move_goal = franka_gripper::MoveGoal();
  move_goal.width = desired_width;
  move_goal.speed = desired_velocity;
  move_client->sendGoal(move_goal);
  bool finished_before_timeout = move_client->waitForResult(ros::Duration(10.0));

  homing_client->sendGoal(franka_gripper::HomingGoal());
  homing_client->waitForResult(ros::Duration(15.));
  updateFingerState();
  EXPECT_TRUE(finished_before_timeout);
  EXPECT_EQ(homing_client->getState(), actionlib::SimpleClientGoalState::SUCCEEDED);
  EXPECT_TRUE(homing_client->getResult()->success);
  EXPECT_NEAR(finger_1_pos * 2, 0.08, kAllowedPositionError);
  EXPECT_NEAR(finger_2_pos * 2, 0.08, kAllowedPositionError);
}

TEST_P(GripperGraspFixtureTest, CanFailGraspGoesToClosedState) {  // NOLINT(cert-err58-cpp)
  double start_width = 0.08;
  auto move_goal = franka_gripper::MoveGoal();
  move_goal.width = start_width;
  move_goal.speed = 0.1;
  this->move_client->sendGoal(move_goal);
  bool finished_before_timeout = move_client->waitForResult(ros::Duration(10.0));
  EXPECT_TRUE(finished_before_timeout);

  double expected_duration = start_width / desired_velocity;
  auto grasp_goal = franka_gripper::GraspGoal();
  grasp_goal.width = desired_width;
  grasp_goal.speed = desired_velocity;
  grasp_goal.force = desired_force;
  grasp_goal.epsilon.inner = 0.005;
  grasp_goal.epsilon.outer = 0.005;
  auto start_time = ros::Time::now();
  this->grasp_client->sendGoal(grasp_goal);
  finished_before_timeout = this->grasp_client->waitForResult(ros::Duration(10.0));
  auto stop_time = ros::Time::now();
  double duration = (stop_time - start_time).toSec();
  updateFingerState();
  EXPECT_TRUE(finished_before_timeout);
  EXPECT_NEAR(finger_1_pos * 2, 0, kAllowedPositionError);
  EXPECT_NEAR(finger_2_pos * 2, 0, kAllowedPositionError);
  EXPECT_NEAR(duration, expected_duration, allowed_absolute_duration_error);
  EXPECT_EQ(grasp_client->getState(), actionlib::SimpleClientGoalState::SUCCEEDED);
  EXPECT_FALSE(grasp_client->getResult()->success);
}

TEST_P(GripperGraspZeroFixtureTest, CanSucceedGraspGoesToClosedState) {  // NOLINT(cert-err58-cpp)
  double start_width = 0.08;
  auto move_goal = franka_gripper::MoveGoal();
  move_goal.width = start_width;
  move_goal.speed = 0.1;
  move_client->sendGoal(move_goal);
  bool finished_before_timeout = move_client->waitForResult(ros::Duration(10.0));
  EXPECT_TRUE(finished_before_timeout);
  ros::Duration(desired_sleep).sleep();

  double expected_duration = start_width / desired_velocity;
  auto grasp_goal = franka_gripper::GraspGoal();
  grasp_goal.width = desired_width;
  grasp_goal.speed = desired_velocity;
  grasp_goal.force = desired_force;
  grasp_goal.epsilon.inner = 0.005;
  grasp_goal.epsilon.outer = 0.005;
  auto start_time = ros::Time::now();
  grasp_client->sendGoal(grasp_goal);
  finished_before_timeout = grasp_client->waitForResult(ros::Duration(10.0));
  auto stop_time = ros::Time::now();
  double duration = (stop_time - start_time).toSec();
  updateFingerState();
  EXPECT_TRUE(finished_before_timeout);
  EXPECT_NEAR(finger_1_pos * 2, 0, kAllowedPositionError);
  EXPECT_NEAR(finger_2_pos * 2, 0, kAllowedPositionError);
  EXPECT_NEAR(duration, expected_duration, allowed_absolute_duration_error);
  EXPECT_EQ(grasp_client->getState(), actionlib::SimpleClientGoalState::SUCCEEDED);
  EXPECT_TRUE(grasp_client->getResult()->success);
}

// TODO(goll_th): Enable again, when not flaky anymore
INSTANTIATE_TEST_CASE_P(DISABLED_GripperMoveFixtureTest,  // NOLINT(cert-err58-cpp)
                        GripperMoveFixtureTest,
                        ::testing::Values(std::make_tuple(0.06, 0.1, 0.1),
                                          std::make_tuple(0.01, 0.1, 0.1),
                                          std::make_tuple(0.07, 0.1, 0.1),
                                          std::make_tuple(0.03, 0.01, 0.6),
                                          std::make_tuple(0.07, 0.01, 0.6),
                                          std::make_tuple(0.0, 0.01, 0.6)));

INSTANTIATE_TEST_CASE_P(  // NOLINT(cert-err58-cpp)
    DISABLED_GripperFailMoveFixtureTest,
    GripperFailMoveFixtureTest,
    ::testing::Values(std::make_tuple(0.09, 0.1, 0),
                      std::make_tuple(std::numeric_limits<double>::quiet_NaN(), 0.1, 0),
                      std::make_tuple(-0.08, 0.1, 0),
                      std::make_tuple(-0.0001, 0.1, 0)));

INSTANTIATE_TEST_CASE_P(DISABLED_GripperHomingFixtureTest,  // NOLINT(cert-err58-cpp)
                        GripperHomingFixtureTest,
                        ::testing::Values(0.06, 0.01, 0.03, 0.08, 0.0));

INSTANTIATE_TEST_CASE_P(DISABLED_GripperGraspFixtureTest,  // NOLINT(cert-err58-cpp)
                        GripperGraspFixtureTest,
                        ::testing::Values(std::make_tuple(0.03, 0.1, 0., 0.1),
                                          std::make_tuple(0.02, 0.1, 5., 0.1),
                                          std::make_tuple(0.03, 0.01, 0., 0.6),
                                          std::make_tuple(0.07, 0.01, 5., 0.6)));

INSTANTIATE_TEST_CASE_P(DISABLED_GripperGraspZeroFixtureTest,  // NOLINT(cert-err58-cpp)
                        GripperGraspZeroFixtureTest,
                        ::testing::Combine(::testing::Values(std::make_tuple(0.0, 0.1, 0., 0.1),
                                                             std::make_tuple(0.0, 0.1, 5., 0.1),
                                                             std::make_tuple(0.0, 0.01, 0., 0.6),
                                                             std::make_tuple(0.0, 0.01, 50., 0.6),
                                                             std::make_tuple(0.0, 0.01, 100., 0.6)),

                                           ::testing::Values(0., 0.1)));

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "franka_gripper_sim_test");
  return RUN_ALL_TESTS();
}
