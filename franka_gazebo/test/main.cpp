// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <gtest/gtest.h>
#include <ros/ros.h>

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "franka_hw_sim_test_node");
  return RUN_ALL_TESTS();
}
