#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <franka_gripper/GraspAction.h>
#include <franka_gripper/HomingAction.h>
#include <franka_gripper/MoveAction.h>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

static const double kAllowedPositionError = 5e-3;
static const double kAllowedRelativeDurationError = 0.2;

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

class GripperGraspFixtureTest
    : public GripperSimTest,
      public testing::WithParamInterface<std::tuple<double, double, double>> {};

class GripperFailGraspFixtureTest
    : public GripperSimTest,
      public testing::WithParamInterface<std::tuple<double, double, double>> {};

/// this one currently fails
// TEST_F(GripperSimTest, FailMove) {  // NOLINT(cert-err58-cpp)
//   double desired_width = 0;
//   double desired_velocity = 0.1;
//
//   auto move_goal = franka_gripper::MoveGoal();
//   move_goal.width = desired_width;
//   move_goal.speed = desired_velocity;
//   move_client->sendGoal(move_goal);
//
//   bool finished_before_timeout = move_client->waitForResult(ros::Duration(15.0));
//   EXPECT_TRUE(finished_before_timeout);
//   EXPECT_TRUE(move_client->getState() == actionlib::SimpleClientGoalState::ABORTED);
//   EXPECT_FALSE(move_client->getResult()->success);
//
//   move_goal.width = 0.08;
//   move_client->sendGoal(move_goal);
//   finished_before_timeout = move_client->waitForResult(ros::Duration(15.0));
//   EXPECT_TRUE(finished_before_timeout);
//   EXPECT_TRUE(move_client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED);
//   EXPECT_TRUE(move_client->getResult()->success);
// }

TEST_P(GripperGraspFixtureTest, CanGraspWithoutDelay) {  // NOLINT(cert-err58-cpp)
  double start_width = 0.08;
  auto move_goal = franka_gripper::MoveGoal();
  move_goal.width = start_width;
  move_goal.speed = 0.1;
  this->move_client->sendGoal(move_goal);
  EXPECT_TRUE(move_client->waitForResult(ros::Duration(10.0)));
  double desired_width = std::get<0>(GetParam());
  double desired_velocity = std::get<1>(GetParam());
  double desired_force = std::get<2>(GetParam());
  const double kStoneWidth = 0.032;
  double expected_duration = (start_width - kStoneWidth) / desired_velocity;
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
  EXPECT_NEAR(finger_1_pos * 2, kStoneWidth, kAllowedPositionError);
  EXPECT_NEAR(finger_2_pos * 2, kStoneWidth, kAllowedPositionError);
  EXPECT_NEAR(duration, expected_duration, expected_duration * kAllowedRelativeDurationError);
  EXPECT_TRUE(grasp_client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED);
  EXPECT_TRUE(grasp_client->getResult()->success);
}

TEST_P(GripperGraspFixtureTest, CanGrasp) {  // NOLINT(cert-err58-cpp)
  double start_width = 0.08;
  auto move_goal = franka_gripper::MoveGoal();
  move_goal.width = start_width;
  move_goal.speed = 0.1;
  this->move_client->sendGoal(move_goal);
  EXPECT_TRUE(move_client->waitForResult(ros::Duration(10.0)));
  ros::Duration(0.1).sleep();
  double desired_width = std::get<0>(GetParam());
  double desired_velocity = std::get<1>(GetParam());
  double desired_force = std::get<2>(GetParam());
  const double kStoneWidth = 0.032;
  double expected_duration = (start_width - kStoneWidth) / desired_velocity;
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
  EXPECT_NEAR(finger_1_pos * 2, kStoneWidth, kAllowedPositionError);
  EXPECT_NEAR(finger_2_pos * 2, kStoneWidth, kAllowedPositionError);
  EXPECT_NEAR(duration, expected_duration, expected_duration * kAllowedRelativeDurationError);
  EXPECT_TRUE(grasp_client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED);
  EXPECT_TRUE(grasp_client->getResult()->success);
}

TEST_P(GripperFailGraspFixtureTest, CanFailGraspWithoutDelay) {  // NOLINT(cert-err58-cpp)
  double start_width = 0.08;
  auto move_goal = franka_gripper::MoveGoal();
  move_goal.width = start_width;
  move_goal.speed = 0.1;
  this->move_client->sendGoal(move_goal);
  this->move_client->waitForResult(ros::Duration(10.0));
  double desired_width = std::get<0>(GetParam());
  double desired_velocity = std::get<1>(GetParam());
  double desired_force = std::get<2>(GetParam());
  const double kStoneWidth = 0.032;
  double expected_duration = (start_width - kStoneWidth) / desired_velocity;
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
  EXPECT_NEAR(finger_1_pos * 2, kStoneWidth, kAllowedPositionError);
  EXPECT_NEAR(finger_2_pos * 2, kStoneWidth, kAllowedPositionError);
  EXPECT_NEAR(duration, expected_duration, expected_duration * kAllowedRelativeDurationError);
  EXPECT_TRUE(grasp_client->getState() == actionlib::SimpleClientGoalState::ABORTED);
  EXPECT_FALSE(grasp_client->getResult()->success);
}

TEST_P(GripperFailGraspFixtureTest, CanFailGrasp) {  // NOLINT(cert-err58-cpp)
  double start_width = 0.08;
  auto move_goal = franka_gripper::MoveGoal();
  move_goal.width = start_width;
  move_goal.speed = 0.1;
  this->move_client->sendGoal(move_goal);
  EXPECT_TRUE(move_client->waitForResult(ros::Duration(10.0)));
  ros::Duration(0.1).sleep();
  double desired_width = std::get<0>(GetParam());
  double desired_velocity = std::get<1>(GetParam());
  double desired_force = std::get<2>(GetParam());
  const double kStoneWidth = 0.032;
  double expected_duration = (start_width - kStoneWidth) / desired_velocity;
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
  EXPECT_NEAR(finger_1_pos * 2, kStoneWidth, kAllowedPositionError);
  EXPECT_NEAR(finger_2_pos * 2, kStoneWidth, kAllowedPositionError);
  EXPECT_NEAR(duration, expected_duration, expected_duration * kAllowedRelativeDurationError);
  EXPECT_TRUE(grasp_client->getState() == actionlib::SimpleClientGoalState::ABORTED);
  EXPECT_FALSE(grasp_client->getResult()->success);
}

INSTANTIATE_TEST_CASE_P(GripperFailGraspFixtureTest,  // NOLINT(cert-err58-cpp)
                        GripperFailGraspFixtureTest,
                        ::testing::Values(std::make_tuple(0.04, 0.1, 0.),
                                          std::make_tuple(0.02, 0.1, 2.)));

INSTANTIATE_TEST_CASE_P(GripperGraspFixtureTest,  // NOLINT(cert-err58-cpp)
                        GripperGraspFixtureTest,
                        ::testing::Values(std::make_tuple(0.032, 0.1, 0.),
                                          std::make_tuple(0.03, 0.1, 5.),
                                          std::make_tuple(0.03, 0.01, 0.),
                                          std::make_tuple(0.034, 0.01, 5.)));

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "franka_gripper_sim_test");
  return RUN_ALL_TESTS();
}
