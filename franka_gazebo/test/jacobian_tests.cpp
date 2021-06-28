#include <gtest/gtest.h>

#include <franka_gazebo/model_kdl.h>
#include <Eigen/Dense>
#include <string>

void compareMatricesElementWise(const Eigen::MatrixXd& expected,
                                const Eigen::MatrixXd& actual,
                                double tolerance) {
  EXPECT_EQ(expected.rows(), actual.rows())
      << "Cannot check element-wise equality, since matrices don't have same rows";
  EXPECT_EQ(expected.cols(), actual.cols())
      << "Cannot check element-wise equality, since matrices don't have same columns";

  for (int r = 0; r < expected.rows(); r++) {
    for (int c = 0; c < expected.rows(); c++) {
      EXPECT_NEAR(expected(r, c), actual(r, c), tolerance)
          << "Element (" << r << "," << c << ") differ!";
    }
  }
}

class JacobianFixture : public ::testing::Test {
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

TEST_F(JacobianFixture, zero_jacobian_joint1_zero_pose) {
  std::array<double, 7> q = {0, 0, 0, 0, 0, 0, 0};
  std::array<double, 42> jacobian =
      model->zeroJacobian(franka::Frame::kJoint1, q, identity, identity);

  Eigen::Matrix<double, 6, 7> expected, actual;
  actual = Eigen::Matrix<double, 6, 7>(jacobian.data());

  // clang-format off
  expected <<
        0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0,
        1, 0, 0, 0, 0, 0, 0;
  // clang-format on
  compareMatricesElementWise(expected, actual, 0.01);
}

TEST_F(JacobianFixture, zero_jacobian_joint2_zero_pose) {
  std::array<double, 7> q = {0, 0, 0, 0, 0, 0, 0};
  std::array<double, 42> jacobian =
      model->zeroJacobian(franka::Frame::kJoint2, q, identity, identity);

  Eigen::Matrix<double, 6, 7> expected, actual;
  actual = Eigen::Matrix<double, 6, 7>(jacobian.data());

  // clang-format off
    expected <<
        0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 0, 0,
        1, 0, 0, 0, 0, 0, 0;
  // clang-format on
  compareMatricesElementWise(expected, actual, 0.01);
}

TEST_F(JacobianFixture, zero_jacobian_joint3_zero_pose) {
  std::array<double, 7> q = {0, 0, 0, 0, 0, 0, 0};
  std::array<double, 42> jacobian =
      model->zeroJacobian(franka::Frame::kJoint3, q, identity, identity);

  Eigen::Matrix<double, 6, 7> expected, actual;
  actual = Eigen::Matrix<double, 6, 7>(jacobian.data());

  // clang-format off
    expected <<
        0, 0.316,  0,  0,  0,  0,  0,
        0,     0,  0,  0,  0,  0,  0,
        0,     0,  0,  0,  0,  0,  0,
        0,     0,  0,  0,  0,  0,  0,
        0,     1,  0,  0,  0,  0,  0,
        1,     0,  1,  0,  0,  0,  0;
  // clang-format on
  compareMatricesElementWise(expected, actual, 0.01);
}

TEST_F(JacobianFixture, zero_jacobian_joint4_zero_pose) {
  std::array<double, 7> q = {0, 0, 0, 0, 0, 0, 0};
  std::array<double, 42> jacobian =
      model->zeroJacobian(franka::Frame::kJoint4, q, identity, identity);

  Eigen::Matrix<double, 6, 7> expected, actual;
  actual = Eigen::Matrix<double, 6, 7>(jacobian.data());

  // clang-format off
    expected <<
        0,   0.316,       0,   0,  0,  0,  0,
   0.0825,       0,  0.0825,   0,  0,  0,  0,
        0, -0.0825,       0,   0,  0,  0,  0,
        0,       0,       0,   0,  0,  0,  0,
        0,       1,       0,  -1,  0,  0,  0,
        1,       0,       1,   0,  0,  0,  0;
  // clang-format on
  compareMatricesElementWise(expected, actual, 0.01);
}

TEST_F(JacobianFixture, zero_jacobian_joint5_zero_pose) {
  std::array<double, 7> q = {0, 0, 0, 0, 0, 0, 0};
  std::array<double, 42> jacobian =
      model->zeroJacobian(franka::Frame::kJoint5, q, identity, identity);

  Eigen::Matrix<double, 6, 7> expected, actual;
  actual = Eigen::Matrix<double, 6, 7>(jacobian.data());

  // clang-format off
  expected <<
      0,     0.7,       0,  -0.384,       0,       0,       0,
      0,       0,       0,       0,       0,       0,       0,
      0,      -0,       0, -0.0825,       0,       0,       0,
      0,      -0,       0,       0,       0,       0,       0,
      0,       1,       0,      -1,       0,       0,       0,
      1,       0,       1,      -0,       1,       0,       0;
  // clang-format on
  compareMatricesElementWise(expected, actual, 0.01);
}

TEST_F(JacobianFixture, zero_jacobian_joint6_zero_pose) {
  std::array<double, 7> q = {0, 0, 0, 0, 0, 0, 0};
  std::array<double, 42> jacobian =
      model->zeroJacobian(franka::Frame::kJoint6, q, identity, identity);

  Eigen::Matrix<double, 6, 7> expected, actual;
  actual = Eigen::Matrix<double, 6, 7>(jacobian.data());

  // clang-format off
    expected <<
        0,     0.7,       0,  -0.384,       0,       0,       0,
        0,       0,       0,       0,       0,       0,       0,
        0,      -0,       0, -0.0825,       0,       0,       0,
        0,      -0,       0,       0,       0,       0,       0,
        0,       1,       0,      -1,       0,      -1,       0,
        1,       0,       1,      -0,       1,       0,       0;
  // clang-format on
  compareMatricesElementWise(expected, actual, 0.01);
}

TEST_F(JacobianFixture, zero_jacobian_joint7_zero_pose) {
  std::array<double, 7> q = {0, 0, 0, 0, 0, 0, 0};
  std::array<double, 42> jacobian =
      model->zeroJacobian(franka::Frame::kJoint7, q, identity, identity);

  Eigen::Matrix<double, 6, 7> expected, actual;
  actual = Eigen::Matrix<double, 6, 7>(jacobian.data());

  // clang-format off
  expected <<
         0,    0.7,      0, -0.384,      0,      0,      0,
     0.088,      0,  0.088,     -0,  0.088,     -0,      0,
         0, -0.088,      0, 0.0055,      0,  0.088,      0,
         0,     -0,      0,      0,      0,      0,      0,
         0,      1,      0,     -1,      0,     -1,      0,
         1,      0,      1,     -0,      1,     -0,     -1;
  // clang-format on
  compareMatricesElementWise(expected, actual, 0.01);
}

TEST_F(JacobianFixture, zero_jacobian_flange_zero_pose) {
  std::array<double, 7> q = {0, 0, 0, 0, 0, 0, 0};
  std::array<double, 42> jacobian =
      model->zeroJacobian(franka::Frame::kFlange, q, identity, identity);

  Eigen::Matrix<double, 6, 7> expected, actual;
  actual = Eigen::Matrix<double, 6, 7>(jacobian.data());

  // clang-format off
  expected <<
        0,  0.593,      0, -0.277,     -0,  0.107,      0,
    0.088,      0,  0.088,     -0,  0.088,      0,      0,
        0, -0.088,      0, 0.0055,      0,  0.088,      0,
        0,     -0,      0,      0,     -0,      0,      0,
        0,      1,      0,     -1,      0,     -1,      0,
        1,      0,      1,     -0,      1,     -0,     -1;
  // clang-format on
  compareMatricesElementWise(expected, actual, 0.01);
}

TEST_F(JacobianFixture, zero_jacobian_flange_elbow_bend_90deg) {
  std::array<double, 7> q = {0, 0, 0, -M_PI / 2, 0, 0, 0};
  std::array<double, 42> jacobian =
      model->zeroJacobian(franka::Frame::kFlange, q, identity, identity);

  Eigen::Matrix<double, 6, 7> expected, actual;
  actual = Eigen::Matrix<double, 6, 7>(jacobian.data());

  // clang-format off
  expected <<
           0,  0.3105,      -0,  0.0055,     -0,  0.088,   0,
      0.3595,       0,  0.3595,       0,  0.088,      0,   0,
           0, -0.3595,       0,   0.277,      0, -0.107,   0,
           0,      -0,       0,       0,      1,      0,  -1,
           0,       1,       0,      -1,      0,     -1,   0,
           1,       0,       1,      -0,      0,     -0,   0;

  // clang-format on
  compareMatricesElementWise(expected, actual, 0.01);
}

TEST_F(JacobianFixture, zero_jacobian_flange_elbow_bend_120deg) {
  std::array<double, 7> q = {0, 0, 0, -120.0 * M_PI / 180.0, 0, 0, 0};
  std::array<double, 42> jacobian =
      model->zeroJacobian(franka::Frame::kFlange, q, identity, identity);

  Eigen::Matrix<double, 6, 7> expected, actual;
  actual = Eigen::Matrix<double, 6, 7>(jacobian.data());

  // clang-format off
  expected <<
          0,    0.172737,           0,    0.143263,           0,   0.0227102,           0,
   0.319639,           0,    0.319639,           0,       0.088,           0,           0,
          0,   -0.319639,           0,    0.237139,           0,   -0.136665,           0,
          0,          -0,           0,           0,    0.866025,           0,   -0.866025,
          0,           1,           0,          -1,           0,          -1,           0,
          1,           0,           1,           0,        -0.5,          -0,         0.5;
  // clang-format on
  compareMatricesElementWise(expected, actual, 0.01);
}

TEST_F(JacobianFixture, zero_jacobian_flange_shoulder_panned_35deg_and_tilted_minus17deg) {
  std::array<double, 7> q = {35 * M_PI / 180.0, -17 * M_PI / 180.0, 0, 0, 0, 0, 0};
  std::array<double, 42> jacobian =
      model->zeroJacobian(franka::Frame::kFlange, q, identity, identity);

  Eigen::Matrix<double, 6, 7> expected, actual;
  actual = Eigen::Matrix<double, 6, 7>(jacobian.data());

  // clang-format off
  expected <<
     0.0511754,     0.485608,   -0.0504747,    -0.218308,   -0.0504747,    0.0627437,            0,
    -0.0730861,     0.340026,    0.0720854,    -0.152861,    0.0720854,    0.0439336,            0,
             0,    0.0892216,            0,   -0.0757273,            0,     0.115439,            0,
             0,    -0.573576,    -0.239497,     0.573576,    -0.239497,     0.573576,     0.239497,
             0,     0.819152,    -0.167698,    -0.819152,    -0.167698,    -0.819152,     0.167698,
             1,            0,     0.956305,            0,     0.956305,            0,    -0.956305;
  // clang-format on
  compareMatricesElementWise(expected, actual, 0.01);
}

TEST_F(JacobianFixture, zero_jacobian_flange_random_pose) {
  std::array<double, 7> q = {0.5157262388785411,  1.2140897359597562,  1.5346381355065786,
                             -3.0398301021734246, -1.2930720893855998, 1.332867311125138,
                             -1.5554459725458225};
  std::array<double, 42> jacobian =
      model->zeroJacobian(franka::Frame::kFlange, q, identity, identity);

  Eigen::Matrix<double, 6, 7> expected, actual;
  actual = Eigen::Matrix<double, 6, 7>(jacobian.data());

  // clang-format off
  expected <<
    -0.114741,   -0.139677,   -0.114265,   -0.134655,   0.0365754,   -0.132448,           0,
    -0.144822,   -0.079183,   0.0803143,    0.425783,   -0.114922,  -0.0392255,           0,
            0,   0.0693989,    0.160459,   0.0177343,   0.0318097,   0.0105776,           0,
            0,   -0.493167,    0.815174,    0.321402,   -0.859908,   -0.293245,    0.608712,
            0,    0.869935,    0.462123,    0.140648,   -0.370783,    0.921392,  -0.0257594,
            1,           0,     0.34919,   -0.936439,   -0.350825,   -0.255036,   -0.792973;
  // clang-format on
  compareMatricesElementWise(expected, actual, 0.01);
}

TEST_F(JacobianFixture, body_jacobian_joint1_zero_pose) {
  std::array<double, 7> q = {0, 0, 0, 0, 0, 0, 0};
  std::array<double, 42> jacobian =
      model->bodyJacobian(franka::Frame::kJoint1, q, identity, identity);

  Eigen::Matrix<double, 6, 7> expected, actual;
  actual = Eigen::Matrix<double, 6, 7>(jacobian.data());

  // clang-format off
  expected <<
      0,    0,    0,    0,    0,    0,    0,
      0,    0,    0,    0,    0,    0,    0,
      0,    0,    0,    0,    0,    0,    0,
      0,    0,    0,    0,    0,    0,    0,
      0,    0,    0,    0,    0,    0,    0,
      1,    0,    0,    0,    0,    0,    0;
  // clang-format on

  compareMatricesElementWise(expected, actual, 0.01);
}

TEST_F(JacobianFixture, body_jacobian_joint2_zero_pose) {
  std::array<double, 7> q = {0, 0, 0, 0, 0, 0, 0};
  std::array<double, 42> jacobian =
      model->bodyJacobian(franka::Frame::kJoint2, q, identity, identity);

  Eigen::Matrix<double, 6, 7> expected, actual;
  actual = Eigen::Matrix<double, 6, 7>(jacobian.data());

  // clang-format off
  expected <<
      0,    0,    0,    0,    0,    0,    0,
      0,    0,    0,    0,    0,    0,    0,
      0,    0,    0,    0,    0,    0,    0,
      0,    0,    0,    0,    0,    0,    0,
     -1,    0,    0,    0,    0,    0,    0,
      0,    1,    0,    0,    0,    0,    0;
  // clang-format on

  compareMatricesElementWise(expected, actual, 0.01);
}

TEST_F(JacobianFixture, body_jacobian_joint3_zero_pose) {
  std::array<double, 7> q = {0, 0, 0, 0, 0, 0, 0};
  std::array<double, 42> jacobian =
      model->bodyJacobian(franka::Frame::kJoint3, q, identity, identity);

  Eigen::Matrix<double, 6, 7> expected, actual;
  actual = Eigen::Matrix<double, 6, 7>(jacobian.data());

  // clang-format off
    expected <<
        0,  0.316,     0,     0,     0,     0,     0,
        0,      0,     0,     0,     0,     0,     0,
        0,      0,     0,     0,     0,     0,     0,
        0,      0,     0,     0,     0,     0,     0,
        0,      1,     0,     0,     0,     0,     0,
        1,      0,     1,     0,     0,     0,     0;
  // clang-format on

  compareMatricesElementWise(expected, actual, 0.01);
}

TEST_F(JacobianFixture, body_jacobian_joint4_zero_pose) {
  std::array<double, 7> q = {0, 0, 0, 0, 0, 0, 0};
  std::array<double, 42> jacobian =
      model->bodyJacobian(franka::Frame::kJoint4, q, identity, identity);

  Eigen::Matrix<double, 6, 7> expected, actual;
  actual = Eigen::Matrix<double, 6, 7>(jacobian.data());

  // clang-format off
    expected <<
          0,   0.316,       0,       0,       0,       0,       0,
          0, -0.0825,       0,       0,       0,       0,       0,
    -0.0825,       0, -0.0825,       0,       0,       0,       0,
          0,       0,       0,       0,       0,       0,       0,
          1,       0,       1,       0,       0,       0,       0,
          0,      -1,       0,       1,       0,       0,       0;
  // clang-format on

  compareMatricesElementWise(expected, actual, 0.01);
}

TEST_F(JacobianFixture, body_jacobian_joint5_zero_pose) {
  std::array<double, 7> q = {0, 0, 0, 0, 0, 0, 0};
  std::array<double, 42> jacobian =
      model->bodyJacobian(franka::Frame::kJoint5, q, identity, identity);

  Eigen::Matrix<double, 6, 7> expected, actual;
  actual = Eigen::Matrix<double, 6, 7>(jacobian.data());

  // clang-format off
    expected <<
          0,     0.7,       0,  -0.384,       0,       0,       0,
          0,       0,       0,       0,       0,       0,       0,
          0,       0,       0, -0.0825,       0,       0,       0,
          0,       0,       0,       0,       0,       0,       0,
          0,       1,       0,      -1,       0,       0,       0,
          1,       0,       1,       0,       1,       0,       0;
  // clang-format on

  compareMatricesElementWise(expected, actual, 0.01);
}

TEST_F(JacobianFixture, body_jacobian_joint6_zero_pose) {
  std::array<double, 7> q = {0, 0, 0, 0, 0, 0, 0};
  std::array<double, 42> jacobian =
      model->bodyJacobian(franka::Frame::kJoint6, q, identity, identity);

  Eigen::Matrix<double, 6, 7> expected, actual;
  actual = Eigen::Matrix<double, 6, 7>(jacobian.data());

  // clang-format off
  expected <<
        0,     0.7,       0,  -0.384,       0,       0,       0,
        0,       0,       0, -0.0825,       0,       0,       0,
        0,       0,       0,       0,       0,       0,       0,
        0,       0,       0,       0,       0,       0,       0,
        1,       0,       1,       0,       1,       0,       0,
        0,      -1,       0,       1,       0,       1,       0;
  // clang-format on

  compareMatricesElementWise(expected, actual, 0.01);
}

TEST_F(JacobianFixture, body_jacobian_joint7_zero_pose) {
  std::array<double, 7> q = {0, 0, 0, 0, 0, 0, 0};
  std::array<double, 42> jacobian =
      model->bodyJacobian(franka::Frame::kJoint7, q, identity, identity);

  Eigen::Matrix<double, 6, 7> expected, actual;
  actual = Eigen::Matrix<double, 6, 7>(jacobian.data());

  // clang-format off
  expected <<
          0,     0.7,       0,  -0.384,       0,       0,       0,
     -0.088,       0,  -0.088,       0,  -0.088,       0,       0,
          0,   0.088,       0, -0.0055,       0,  -0.088,       0,
          0,       0,       0,       0,       0,       0,       0,
          0,      -1,       0,       1,       0,       1,       0,
         -1,       0,      -1,       0,      -1,       0,       1;
  // clang-format on

  compareMatricesElementWise(expected, actual, 0.01);
}

TEST_F(JacobianFixture, body_jacobian_flange_zero_pose) {
  std::array<double, 7> q = {0, 0, 0, 0, 0, 0, 0};
  std::array<double, 42> jacobian =
      model->bodyJacobian(franka::Frame::kFlange, q, identity, identity);

  Eigen::Matrix<double, 6, 7> expected, actual;
  actual = Eigen::Matrix<double, 6, 7>(jacobian.data());

  // clang-format off
  expected <<
        0,   0.593,       0,  -0.277,       0,   0.107,       0,
   -0.088,       0,  -0.088,       0,  -0.088,       0,       0,
        0,   0.088,       0, -0.0055,       0,  -0.088,       0,
        0,       0,       0,       0,       0,       0,       0,
        0,      -1,       0,       1,       0,       1,       0,
       -1,       0,      -1,       0,      -1,       0,       1;
  // clang-format on

  compareMatricesElementWise(expected, actual, 0.01);
}

TEST_F(JacobianFixture, body_jacobian_flange_elbow_bend_90deg) {
  std::array<double, 7> q = {0, 0, 0, -M_PI / 2, 0, 0, 0};
  std::array<double, 42> jacobian =
      model->bodyJacobian(franka::Frame::kFlange, q, identity, identity);

  Eigen::Matrix<double, 6, 7> expected, actual;
  actual = Eigen::Matrix<double, 6, 7>(jacobian.data());

  // clang-format off
  expected <<
           0,    0.3595,         0,    -0.277,        0,    0.107,     0,
     -0.3595,         0,   -0.3595,         0,   -0.088,        0,     0,
           0,   -0.3105,         0,   -0.0055,        0,   -0.088,     0,
          -1,         0,        -1,         0,        0,        0,     0,
           0,        -1,         0,         1,        0,        1,     0,
           0,         0,         0,         0,       -1,        0,     1;
  // clang-format on

  compareMatricesElementWise(expected, actual, 0.01);
}

TEST_F(JacobianFixture, body_jacobian_flange_elbow_bend_120deg) {
  std::array<double, 7> q = {0, 0, 0, -120.0 * M_PI / 180.0, 0, 0, 0};
  std::array<double, 42> jacobian =
      model->bodyJacobian(franka::Frame::kFlange, q, identity, identity);

  Eigen::Matrix<double, 6, 7> expected, actual;
  actual = Eigen::Matrix<double, 6, 7>(jacobian.data());

  // clang-format off
  expected <<
         0,     0.1904,          0,     -0.277,         0,     0.107,       0,
   -0.3196,          0,    -0.3196,          0,    -0.088,         0,       0,
         0,    -0.3094,          0,    -0.0055,         0,    -0.088,       0,
   -0.8660,          0,    -0.8660,          0,         0,         0,       0,
         0,         -1,          0,          1,         0,         1,       0,
       0.5,          0,        0.5,          0,        -1,        -0,       1;
  // clang-format on

  compareMatricesElementWise(expected, actual, 0.01);
}

TEST_F(JacobianFixture, body_jacobian_flange_shoulder_panned_35deg_and_tilted_minus17deg) {
  std::array<double, 7> q = {35 * M_PI / 180.0, -17 * M_PI / 180.0, 0, 0, 0, 0, 0};
  std::array<double, 42> jacobian =
      model->bodyJacobian(franka::Frame::kFlange, q, identity, identity);

  Eigen::Matrix<double, 6, 7> expected, actual;
  actual = Eigen::Matrix<double, 6, 7>(jacobian.data());

  // clang-format off
  expected <<
          0,     0.593,         0,    -0.277,        0,    0.107,      0,
     0.0892,        -0,    -0.088,         0,   -0.088,        0,      0,
          0,     0.088,         0,   -0.0055,        0,   -0.088,      0,
     0.2923,         0,         0,         0,        0,        0,      0,
         -0,        -1,         0,         1,        0,        1,      0,
    -0.9563,         0,        -1,         0,       -1,        0,      1;
  // clang-format on

  compareMatricesElementWise(expected, actual, 0.01);
}

TEST_F(JacobianFixture, body_jacobian_flange_random_pose) {
  std::array<double, 7> q = {0.5157262388785411,  1.2140897359597562,  1.5346381355065786,
                             -3.0398301021734246, -1.2930720893855998, 1.332867311125138,
                             -1.5554459725458225};
  std::array<double, 42> jacobian =
      model->bodyJacobian(franka::Frame::kFlange, q, identity, identity);

  Eigen::Matrix<double, 6, 7> expected, actual;
  actual = Eigen::Matrix<double, 6, 7>(jacobian.data());

  // clang-format off
  expected <<
      0.101939,    0.051154,    -0.067125,   -0.42838,   0.12471,   0.00164,     0,
      0.139199,    0.094503,    -0.034664,   -0.06908,  -0.00191,   0.10698,     0,
     -0.066113,   -0.138015,    -0.198863,   -0.10699,   0      ,  -0.088  ,     0,
      0.246513,   -0.945656,    -0.112624,   -0.27065,   0.01491,  -0.99988,     0,
     -0.557158,    0.040744,    -0.971751,    0.23084,   0.97171,   0.01534,     0,
     -0.792973,   -0.322605,     0.207404,    0.93459,  -0.23569,   0      ,     1;
  // clang-format on

  compareMatricesElementWise(expected, actual, 0.01);
}
