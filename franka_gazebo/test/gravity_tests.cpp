#include <gtest/gtest.h>

#include <franka_gazebo/model_kdl.h>
#include <Eigen/Dense>
#include <string>

class GravityFixture : public ::testing::Test {
 protected:
  std::unique_ptr<franka_hw::ModelBase> model;
  std::array<double, 3> F_x_Ctotal;
  double m_total = 0;

  virtual void SetUp() {
    F_x_Ctotal = {0, 0, 0};
    urdf::Model robot;
    robot.initParam("robot_description");
    EXPECT_GE(robot.joints_.size(), 7);
    model = std::make_unique<franka_gazebo::ModelKDL>(robot, "panda_link0", "panda_link8");
  }

  virtual void TearDown() {}
};

void compareVectorsElementWise(const Eigen::VectorXd& expected,
                               const Eigen::VectorXd& actual,
                               double tolerance) {
  EXPECT_EQ(expected.size(), actual.size())
      << "Cannot check element-wise equality, since vectors don't have same sime";

  for (int i = 0; i < expected.size(); i++) {
    EXPECT_NEAR(expected(i), actual(i), tolerance) << "Element (" << i << ") differs!";
  }
}

TEST_F(GravityFixture, gravity_default_zero_pose) {
  std::array<double, 7> q = {0, 0, 0, 0, 0, 0, 0};
  std::array<double, 7> g = model->gravity(q, m_total, F_x_Ctotal);

  Eigen::Matrix<double, 7, 1> expected;
  expected << 0, -3.47218, 0, -3.20896, 0, 1.68623, 0;

  Eigen::Matrix<double, 7, 1> actual(g.data());

  // NOTE: Tolerance has to be quite high, since inertia values are only estimated
  compareVectorsElementWise(expected, actual, 0.05);
}

TEST_F(GravityFixture, gravity_default_random_pose) {
  std::array<double, 7> q = {0.5157262388785411,  1.2140897359597562,  1.5346381355065786,
                             -3.0398301021734246, -1.2930720893855998, 1.332867311125138,
                             -1.5554459725458225};
  std::array<double, 7> g = model->gravity(q, m_total, F_x_Ctotal);

  Eigen::Matrix<double, 7, 1> expected;
  expected << 0, -12.1212, 11.5316, 1.35018, 0.973167, 0.918118, -0.0330053;

  Eigen::Matrix<double, 7, 1> actual(g.data());

  // NOTE: Tolerance has to be quite high, since inertia values are only estimated
  compareVectorsElementWise(expected, actual, 0.1);
}

TEST_F(GravityFixture, gravity_sideways_zero_pose) {
  std::array<double, 7> q = {0, 0, 0, 0, 0, 0, 0};
  std::array<double, 7> g = model->gravity(q, m_total, F_x_Ctotal, {0, +9.81, 0});

  Eigen::Matrix<double, 7, 1> expected;
  expected << -3.87262, 0, -3.52832, 0, -1.587733, 0, 0.0612622;

  Eigen::Matrix<double, 7, 1> actual(g.data());

  // NOTE: Tolerance has to be quite high, since inertia values are only estimated
  compareVectorsElementWise(expected, actual, 0.25);
}

TEST_F(GravityFixture, gravity_sideways_random_pose) {
  std::array<double, 7> q = {0.5157262388785411,  1.2140897359597562,  1.5346381355065786,
                             -3.0398301021734246, -1.2930720893855998, 1.332867311125138,
                             -1.5554459725458225};
  std::array<double, 7> g = model->gravity(q, m_total, F_x_Ctotal, {0, +9.81, 0});

  Eigen::Matrix<double, 7, 1> expected;
  expected << -5.46378, -1.79291, 1.35189, -18.1651, 0.761036, 0.215834, 0.0271278;

  Eigen::Matrix<double, 7, 1> actual(g.data());

  // NOTE: Tolerance has to be quite high, since inertia values are only estimated
  compareVectorsElementWise(expected, actual, 1.0);
}
