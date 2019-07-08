// Copyright (c) 2019 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <franka_hw/franka_hw.h>

namespace franka_hw {

class FrankaCombinableHW : public FrankaHW {
  /**
   * Creates an instance of FrankaCombinableHW.
   *
   */
  FrankaCombinableHW();

  // FrankaHW::initParameters ()
  // FrankaHW::init()

  void initROSInterfaces() override;
};

}  // namespace franka_hw
