# ROS integration for FRANKA EMIKA research robots

See the [FRANKA Control Interface documentation][fci-docs] for more information.

## MoveIt! quickstart

1. Launch the joint trajectory controller

    roslaunch franka_control franka_control.launch robot_ip:=<ip>

2. Launch MoveIt!

    roslaunch franka_moveit_config franka_moveit.launch

3. Launch RViz

    roslaunch franka_moveit_config moveit_rviz.launch
