# CHANGELOG

## 0.2.0 - 2018-01-29

Requires `libfranka` >= 0.2.0

  * Added missing run-time dependencies to `franka_description` and `franka_control`
  * Added `tau_J_d`, `m_ee`, `F_x_Cee`, `I_ee`, `m_total`, `F_x_Ctotal`, `I_total`,
    `theta` and `dtheta` to `franka_msgs/FrankaState`
  * Added new errors to `franka_msgs/Errors`
  * Updated and improved examples in `franka_example_controllers`
  * Fixed includes for Eigen3 in `franka_example_controllers`
  * Fixed gripper state publishing in `franka_gripper_node`

## 0.1.2 - 2017-10-10

  * Fixed out-of-workspace build

## 0.1.1 - 2017-10-09

  * Integrated `franka_description` as subdirectory
  * Fixed dependencies on libfranka
  * Fixed RViz config file paths
  * Added missing `test_depend` to `franka_hw`
  * Added missing CMake install rules

## 0.1.0 - 2017-09-15

  * Initial release

