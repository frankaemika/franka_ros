// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <ros/time.h>

namespace franka_hw {

class TriggerRate {
 public:
  explicit TriggerRate(double rate = 30.0);
  bool operator()();

 private:
  ros::Time time_stamp_;
  double period_;
};

};  // namespace franka_hw
