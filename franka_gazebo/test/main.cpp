// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <gtest/gtest.h>
#include <ros/ros.h>

#include <franka/robot_state.h>
#include <libfcimodels.h>
#include <Eigen/Dense>
#include <string>

void printJac() {
  double array_O_J_Ji[42];
  double O_J_Ji[42];

  double q[] = {0.5157262388785411,  1.2140897359597562, 1.5346381355065786, -3.0398301021734246,
                -1.2930720893855998, 1.332867311125138,  -1.5554459725458225};
  O_J_J8(q, O_J_Ji);

  Eigen::Matrix<double, 6, 7> jacobian(O_J_Ji);
  std::cout << jacobian << std::endl;
}

void printTf() {
  double out[16];
  // double q[] = {0, 0, 0, 0, 0, 0, 0};

  double q[] = {0.5157262388785411,  1.2140897359597562, 1.5346381355065786, -3.0398301021734246,
                -1.2930720893855998, 1.332867311125138,  -1.5554459725458225};
  O_T_J8(q, out);

  Eigen::Affine3d transform(Eigen::Matrix4d::Map(out));
  std::cout << Eigen::Matrix4d::Map(out) << std::endl;
  // Eigen::Vector3d position(transform.translation());
  // Eigen::Quaterniond orientation(transform.linear());

  // std::cout << position(0) << " " << position(1) << " " << position(2) << std::endl;
  // std::cout << orientation.x() << " " << orientation.y() << " " << orientation.z() << " "
  //           << orientation.w() << std::endl;
}

int main(int argc, char** argv) {
  // printJac();
  printTf();
  // return 0;

  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "franka_hw_sim_test_node");
  return RUN_ALL_TESTS();
}
