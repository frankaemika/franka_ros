#include <gtest/gtest.h>

#include <franka_gazebo/model_kdl.h>
#include <Eigen/Dense>
#include <string>

class FkFixture : public ::testing::Test {
 protected:
  std::unique_ptr<franka_gazebo::ModelKDL> model;
  std::array<double, 16> identity;

  virtual void SetUp() {
    identity = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
    urdf::Model robot;
    robot.initParam("robot_description");
    EXPECT_GE(robot.joints_.size(), 7);
    model = std::make_unique<franka_gazebo::ModelKDL>(robot, "panda_link0", "panda_link8");
  }

  virtual void TearDown() {}
};

void compareAffineTransformations(const Eigen::Affine3d& expected,
                                  const Eigen::Affine3d& actual,
                                  double tolerance) {
  EXPECT_NEAR(Eigen::Vector3d(expected.translation())(0), Eigen::Vector3d(actual.translation())(0),
              tolerance);
  EXPECT_NEAR(Eigen::Vector3d(expected.translation())(1), Eigen::Vector3d(actual.translation())(1),
              tolerance);
  EXPECT_NEAR(Eigen::Vector3d(expected.translation())(2), Eigen::Vector3d(actual.translation())(2),
              tolerance);

  EXPECT_NEAR(Eigen::Quaterniond(expected.linear()).x(), Eigen::Quaterniond(actual.linear()).x(),
              tolerance);
  EXPECT_NEAR(Eigen::Quaterniond(expected.linear()).y(), Eigen::Quaterniond(actual.linear()).y(),
              tolerance);
  EXPECT_NEAR(Eigen::Quaterniond(expected.linear()).z(), Eigen::Quaterniond(actual.linear()).z(),
              tolerance);
  EXPECT_NEAR(Eigen::Quaterniond(expected.linear()).w(), Eigen::Quaterniond(actual.linear()).w(),
              tolerance);
}

TEST_F(FkFixture, fk_joint1_zero_pose) {
  std::array<double, 7> q = {0, 0, 0, 0, 0, 0, 0};
  std::array<double, 16> pose = model->pose(franka::Frame::kJoint1, q, identity, identity);

  Eigen::Matrix4d expected;
  // clang-format off
  expected <<
      1,    0,    0,       0,
      0,    1,    0,       0,
      0,    0,    1,   0.333,
      0,    0,    0,       1;
  // clang-format on

  Eigen::Affine3d actual(Eigen::Matrix4d(pose.data()));
  compareAffineTransformations(Eigen::Affine3d(expected), actual, 0.001);
}

TEST_F(FkFixture, fk_joint2_zero_pose) {
  std::array<double, 7> q = {0, 0, 0, 0, 0, 0, 0};
  std::array<double, 16> pose = model->pose(franka::Frame::kJoint2, q, identity, identity);

  Eigen::Matrix4d expected;
  // clang-format off
  expected <<
      1,    0,    0,       0,
      0,    0,    1,       0,
      0,   -1,    0,   0.333,
      0,    0,    0,       1;
  // clang-format on

  Eigen::Affine3d actual(Eigen::Matrix4d(pose.data()));
  compareAffineTransformations(Eigen::Affine3d(expected), actual, 0.001);
}

TEST_F(FkFixture, fk_joint3_zero_pose) {
  std::array<double, 7> q = {0, 0, 0, 0, 0, 0, 0};
  std::array<double, 16> pose = model->pose(franka::Frame::kJoint3, q, identity, identity);

  Eigen::Matrix4d expected;
  // clang-format off
  expected <<
      1,    0,    0,       0,
      0,    1,    0,       0,
      0,    0,    1,   0.649,
      0,    0,    0,       1;
  // clang-format on

  Eigen::Affine3d actual(Eigen::Matrix4d(pose.data()));
  compareAffineTransformations(Eigen::Affine3d(expected), actual, 0.001);
}

TEST_F(FkFixture, fk_joint4_zero_pose) {
  std::array<double, 7> q = {0, 0, 0, 0, 0, 0, 0};
  std::array<double, 16> pose = model->pose(franka::Frame::kJoint4, q, identity, identity);

  Eigen::Matrix4d expected;
  // clang-format off
  expected <<
      1,    0,    0,  0.0825,
      0,    0,   -1,       0,
      0,    1,    0,   0.649,
      0,    0,    0,       1;
  // clang-format on

  Eigen::Affine3d actual(Eigen::Matrix4d(pose.data()));
  compareAffineTransformations(Eigen::Affine3d(expected), actual, 0.001);
}

TEST_F(FkFixture, fk_joint5_zero_pose) {
  std::array<double, 7> q = {0, 0, 0, 0, 0, 0, 0};
  std::array<double, 16> pose = model->pose(franka::Frame::kJoint5, q, identity, identity);

  Eigen::Matrix4d expected;
  // clang-format off
  expected <<
      1,    0,    0,       0,
      0,    1,    0,       0,
      0,    0,    1,   1.033,
      0,    0,    0,       1;
  // clang-format on

  Eigen::Affine3d actual(Eigen::Matrix4d(pose.data()));
  compareAffineTransformations(Eigen::Affine3d(expected), actual, 0.001);
}

TEST_F(FkFixture, fk_joint6_zero_pose) {
  std::array<double, 7> q = {0, 0, 0, 0, 0, 0, 0};
  std::array<double, 16> pose = model->pose(franka::Frame::kJoint6, q, identity, identity);

  Eigen::Matrix4d expected;
  // clang-format off
  expected <<
      1,    0,    0,       0,
      0,    0,   -1,       0,
      0,    1,    0,   1.033,
      0,    0,    0,       1;
  // clang-format on

  Eigen::Affine3d actual(Eigen::Matrix4d(pose.data()));
  compareAffineTransformations(Eigen::Affine3d(expected), actual, 0.001);
}

TEST_F(FkFixture, fk_joint7_zero_pose) {
  std::array<double, 7> q = {0, 0, 0, 0, 0, 0, 0};
  std::array<double, 16> pose = model->pose(franka::Frame::kJoint7, q, identity, identity);

  Eigen::Matrix4d expected;
  // clang-format off
  expected <<
      1,    0,    0,   0.088,
      0,   -1,    0,       0,
      0,    0,   -1,   1.033,
      0,    0,    0,       1;
  // clang-format on

  Eigen::Affine3d actual(Eigen::Matrix4d(pose.data()));
  compareAffineTransformations(Eigen::Affine3d(expected), actual, 0.001);
}

TEST_F(FkFixture, fk_flange_zero_pose) {
  std::array<double, 7> q = {0, 0, 0, 0, 0, 0, 0};
  std::array<double, 16> pose = model->pose(franka::Frame::kFlange, q, identity, identity);

  Eigen::Matrix4d expected;
  // clang-format off
  expected <<
      1,    0,    0,   0.088,
      0,   -1,    0,       0,
      0,    0,   -1,   0.926,
      0,    0,    0,       1;
  // clang-format on

  Eigen::Affine3d actual(Eigen::Matrix4d(pose.data()));
  compareAffineTransformations(Eigen::Affine3d(expected), actual, 0.001);
}

TEST_F(FkFixture, fk_flange_random_pose) {
  std::array<double, 7> q = {0.5157262388785411,  1.2140897359597562,  1.5346381355065786,
                             -3.0398301021734246, -1.2930720893855998, 1.332867311125138,
                             -1.5554459725458225};
  std::array<double, 16> pose = model->pose(franka::Frame::kFlange, q, identity, identity);

  Eigen::Matrix4d expected;
  // clang-format off
    expected <<
        0.281895,  -0.741623,   0.608712,  -0.144822,
       -0.927236,   -0.37359, -0.0257594,   0.114741,
        0.246513,  -0.557158,  -0.792973,    0.17244,
               0,          0,          0,          1;
  // clang-format on

  Eigen::Affine3d actual(Eigen::Matrix4d(pose.data()));
  compareAffineTransformations(Eigen::Affine3d(expected), actual, 0.001);
}
