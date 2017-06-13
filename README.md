# MoveIt! for FRANKA EMIKA

## Launch the joint trajectory controllers

This assumes that the robot is reachable at robot.franka.de. If this is not the case, add `robot_ip:=<ip>`.

    roslaunch franka_hw franka_hw.launch

## Launch MoveIt! and RViz

    roslaunch franka_moveit_config franka_moveit.launch
    roslaunch franka_moveit_config moveit_rviz.launch
