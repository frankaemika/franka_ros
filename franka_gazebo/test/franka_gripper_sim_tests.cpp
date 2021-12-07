#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <franka_gripper/GraspAction.h>
#include <franka_gripper/HomingAction.h>
#include <franka_gripper/MoveAction.h>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

static const double kAllowedPositionError = 5e-3;
static const double kAllowedRelativeDurationError = 0.1;

class GripperSimTest : public ::testing::Test {
 protected:
  std::unique_ptr<actionlib::SimpleActionClient<franka_gripper::HomingAction>> homing_client;
  std::unique_ptr<actionlib::SimpleActionClient<franka_gripper::MoveAction>> move_client;
  std::unique_ptr<actionlib::SimpleActionClient<franka_gripper::GraspAction>> grasp_client;
  ros::Subscriber joint_states_sub;
  ros::NodeHandle n;
  double finger_1_pos = 0.;
  double finger_2_pos = 0.;
  bool got_joint_states = false;

  void SetUp() override {
    joint_states_sub = n.subscribe<sensor_msgs::JointState>(
        "/franka_gripper/joint_states", 1, [this](const sensor_msgs::JointState::ConstPtr& msg) {
          finger_1_pos = msg->position.at(0);
          finger_2_pos = msg->position.at(1);
          got_joint_states = true;
        });
    homing_client = std::make_unique<actionlib::SimpleActionClient<franka_gripper::HomingAction>>(
        "franka_gripper/homing", true);
    move_client = std::make_unique<actionlib::SimpleActionClient<franka_gripper::MoveAction>>(
        "franka_gripper/move", true);
    grasp_client = std::make_unique<actionlib::SimpleActionClient<franka_gripper::GraspAction>>(
        "franka_gripper/grasp", true);

    homing_client->waitForServer();
    move_client->waitForServer();
    grasp_client->waitForServer();
    while (not got_joint_states) {
      ros::spinOnce();
      ros::Duration(0.1).sleep();
    }
  }
  void TearDown() override { joint_states_sub.shutdown(); }
};

class MoveData : public GripperSimTest,
                 public testing::TestWithParam<std::tuple<double, double>> {};

class GripperMoveFixtureTest : public GripperSimTest,
                               public testing::WithParamInterface<std::tuple<double, double>> {};

class GripperGraspFixtureTest
    : public GripperSimTest,
      public testing::WithParamInterface<std::tuple<double, double, double>> {};

class GripperGraspZeroFixtureTest
    : public GripperSimTest,
      public testing::WithParamInterface<std::tuple<double, double, double>> {};

class GripperFailMoveFixtureTest : public GripperSimTest,
                                   public testing::WithParamInterface<std::tuple<double, double>> {
};

class GripperHomingFixtureTest : public GripperSimTest,
                                 public testing::WithParamInterface<double> {};

TEST_P(GripperFailMoveFixtureTest, FailMove) {  // NOLINT(cert-err58-cpp)
  double desired_width = std::get<0>(GetParam());
  double desired_velocity = std::get<1>(GetParam());

  auto move_goal = franka_gripper::MoveGoal();
  move_goal.width = desired_width;
  move_goal.speed = desired_velocity;
  move_client->sendGoal(move_goal);

  bool finished_before_timeout = move_client->waitForResult(ros::Duration(15.0));
  EXPECT_TRUE(finished_before_timeout);
  EXPECT_TRUE(move_client->getState() == actionlib::SimpleClientGoalState::ABORTED);
  EXPECT_FALSE(move_client->getResult()->success);
}

TEST_P(GripperMoveFixtureTest, CanPerformMove) {  // NOLINT(cert-err58-cpp)
  ros::spinOnce();
  double desired_width = std::get<0>(GetParam());
  double desired_velocity = std::get<1>(GetParam());

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
  ros::spinOnce();

  EXPECT_TRUE(finished_before_timeout);
  EXPECT_TRUE(move_client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED);
  EXPECT_TRUE(move_client->getResult()->success);
  EXPECT_NEAR(finger_1_pos * 2, desired_width, kAllowedPositionError);
  EXPECT_NEAR(finger_2_pos * 2, desired_width, kAllowedPositionError);
  EXPECT_NEAR(duration, expected_duration, expected_duration * kAllowedRelativeDurationError);
}

TEST_P(GripperHomingFixtureTest, CanPerformHoming) {  // NOLINT(cert-err58-cpp)
  double desired_width = GetParam();
  double desired_velocity = 0.01;

  auto move_goal = franka_gripper::MoveGoal();

  move_goal.width = desired_width;
  move_goal.speed = desired_velocity;

  move_client->sendGoal(move_goal);
  bool finished_before_timeout = move_client->waitForResult(ros::Duration(10.0));

  homing_client->sendGoal(franka_gripper::HomingGoal());
  homing_client->waitForResult(ros::Duration(15.));
  ros::spinOnce();
  EXPECT_TRUE(finished_before_timeout);
  EXPECT_TRUE(homing_client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED);
  EXPECT_TRUE(homing_client->getResult()->success);
  EXPECT_NEAR(finger_1_pos * 2, 0.08, kAllowedPositionError);
  EXPECT_NEAR(finger_2_pos * 2, 0.08, kAllowedPositionError);
}

TEST_P(GripperGraspFixtureTest, CanFailGraspGoToClosedState) {  // NOLINT(cert-err58-cpp)
  double start_width = 0.08;
  auto move_goal = franka_gripper::MoveGoal();
  move_goal.width = start_width;
  move_goal.speed = 0.1;
  this->move_client->sendGoal(move_goal);
  this->move_client->waitForResult(ros::Duration(10.0));
  double desired_width = std::get<0>(GetParam());
  double desired_velocity = std::get<1>(GetParam());
  double desired_force = std::get<2>(GetParam());

  double expected_duration = start_width / desired_velocity;
  auto grasp_goal = franka_gripper::GraspGoal();
  grasp_goal.width = desired_width;
  grasp_goal.speed = desired_velocity;
  grasp_goal.force = desired_force;
  grasp_goal.epsilon.inner = 0.005;
  grasp_goal.epsilon.outer = 0.005;
  auto start_time = ros::Time::now();
  this->grasp_client->sendGoal(grasp_goal);
  bool finished_before_timeout = this->grasp_client->waitForResult(ros::Duration(10.0));
  auto stop_time = ros::Time::now();
  double duration = (stop_time - start_time).toSec();
  ros::spinOnce();
  EXPECT_TRUE(finished_before_timeout);
  EXPECT_NEAR(finger_1_pos * 2, 0, kAllowedPositionError);
  EXPECT_NEAR(finger_2_pos * 2, 0, kAllowedPositionError);
  EXPECT_NEAR(duration, expected_duration, expected_duration * kAllowedRelativeDurationError);
  EXPECT_TRUE(grasp_client->getState() == actionlib::SimpleClientGoalState::ABORTED);
  EXPECT_FALSE(grasp_client->getResult()->success);
}

TEST_P(GripperGraspZeroFixtureTest, CanSucceedGraspGoToClosedState) {  // NOLINT(cert-err58-cpp)
  double start_width = 0.08;
  auto move_goal = franka_gripper::MoveGoal();
  move_goal.width = start_width;
  move_goal.speed = 0.1;
  move_client->sendGoal(move_goal);
  EXPECT_TRUE(move_client->waitForResult(ros::Duration(10.0)));
  ros::Duration(0.1).sleep();
  double desired_width = std::get<0>(GetParam());
  double desired_velocity = std::get<1>(GetParam());
  double desired_force = std::get<2>(GetParam());

  double expected_duration = start_width / desired_velocity;
  auto grasp_goal = franka_gripper::GraspGoal();
  grasp_goal.width = desired_width;
  grasp_goal.speed = desired_velocity;
  grasp_goal.force = desired_force;
  grasp_goal.epsilon.inner = 0.005;
  grasp_goal.epsilon.outer = 0.005;
  auto start_time = ros::Time::now();
  grasp_client->sendGoal(grasp_goal);
  bool finished_before_timeout = grasp_client->waitForResult(ros::Duration(10.0));
  auto stop_time = ros::Time::now();
  double duration = (stop_time - start_time).toSec();
  ros::spinOnce();
  EXPECT_TRUE(finished_before_timeout);
  EXPECT_NEAR(finger_1_pos * 2, 0, kAllowedPositionError);
  EXPECT_NEAR(finger_2_pos * 2, 0, kAllowedPositionError);
  EXPECT_NEAR(duration, expected_duration, expected_duration * kAllowedRelativeDurationError);
  EXPECT_TRUE(grasp_client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED);
  EXPECT_TRUE(grasp_client->getResult()->success);
}

// NOLINTNEXTLINE(cert-err58-cpp)
TEST_P(GripperGraspZeroFixtureTest, CanSucceedGraspGoToClosedStateWithoutDelay) {
  double start_width = 0.08;
  auto move_goal = franka_gripper::MoveGoal();
  move_goal.width = start_width;
  move_goal.speed = 0.1;
  move_client->sendGoal(move_goal);
  EXPECT_TRUE(move_client->waitForResult(ros::Duration(10.0)));

  double desired_width = std::get<0>(GetParam());
  double desired_velocity = std::get<1>(GetParam());
  double desired_force = std::get<2>(GetParam());

  double expected_duration = start_width / desired_velocity;
  auto grasp_goal = franka_gripper::GraspGoal();
  grasp_goal.width = desired_width;
  grasp_goal.speed = desired_velocity;
  grasp_goal.force = desired_force;
  grasp_goal.epsilon.inner = 0.005;
  grasp_goal.epsilon.outer = 0.005;
  auto start_time = ros::Time::now();
  grasp_client->sendGoal(grasp_goal);
  bool finished_before_timeout = grasp_client->waitForResult(ros::Duration(10.0));
  auto stop_time = ros::Time::now();
  double duration = (stop_time - start_time).toSec();
  ros::spinOnce();
  EXPECT_TRUE(finished_before_timeout);
  EXPECT_NEAR(finger_1_pos * 2, 0, kAllowedPositionError);
  EXPECT_NEAR(finger_2_pos * 2, 0, kAllowedPositionError);
  EXPECT_NEAR(duration, expected_duration, expected_duration * kAllowedRelativeDurationError);
  EXPECT_TRUE(grasp_client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED);
  EXPECT_TRUE(grasp_client->getResult()->success);
}

INSTANTIATE_TEST_CASE_P(GripperMoveFixtureTest,  // NOLINT(cert-err58-cpp)
                        GripperMoveFixtureTest,
                        ::testing::Values(std::make_tuple(0.06, 0.1),
                                          std::make_tuple(0.01, 0.1),
                                          std::make_tuple(0.07, 0.1),
                                          std::make_tuple(0.03, 0.01),
                                          std::make_tuple(0.07, 0.01),
                                          std::make_tuple(0.0, 0.01)));

INSTANTIATE_TEST_CASE_P(  // NOLINT(cert-err58-cpp)
    GripperFailMoveFixtureTest,
    GripperFailMoveFixtureTest,
    ::testing::Values(std::make_tuple(0.09, 0.1),
                      // std::make_tuple(std::numeric_limits<double>::quiet_NaN(), 0.1),
                      std::make_tuple(-0.08, 0.1),
                      std::make_tuple(-0.0001, 0.1)));

INSTANTIATE_TEST_CASE_P(GripperHomingFixtureTest,  // NOLINT(cert-err58-cpp)
                        GripperHomingFixtureTest,
                        ::testing::Values(0.06, 0.01, 0.03, 0.08, 0.0));

INSTANTIATE_TEST_CASE_P(GripperGraspFixtureTest,  // NOLINT(cert-err58-cpp)
                        GripperGraspFixtureTest,
                        ::testing::Values(std::make_tuple(0.03, 0.1, 0.),
                                          std::make_tuple(0.02, 0.1, 5.),
                                          std::make_tuple(0.03, 0.01, 0.),
                                          std::make_tuple(0.07, 0.01, 5.)));

INSTANTIATE_TEST_CASE_P(GripperGraspZeroFixtureTest,  // NOLINT(cert-err58-cpp)
                        GripperGraspZeroFixtureTest,
                        ::testing::Values(std::make_tuple(0.0, 0.1, 0.),
                                          std::make_tuple(0.0, 0.1, 5.),
                                          std::make_tuple(0.0, 0.01, 0.),
                                          std::make_tuple(0.0, 0.01, 50.),
                                          std::make_tuple(0.0, 0.01, 100.)));

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "franka_gripper_sim_test");
  return RUN_ALL_TESTS();
}
