# ROS integration for Franka Emika research robots

See the [Franka Control Interface (FCI) documentation][fci-docs] for more information. 

## MoveIt! quickstart for Panda research

1. Launch the joint trajectory controller

```
roslaunch franka_control franka_control.launch robot_ip:=<ip>
```

2. Launch MoveIt!

```
roslaunch panda_moveit_config panda_moveit.launch
```

3. Launch RViz

```
roslaunch panda_moveit_config moveit_rviz.launch
```

[fci-docs]: https://frankaemika.github.io/docs
