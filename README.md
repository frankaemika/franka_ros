# ROS integration for Franka Emika research robots

[![Build Status][travis-status]][travis]

See the [Franka Control Interface (FCI) documentation][fci-docs] for more information.

## License

All packages of `franka_ros` are licensed under the [Apache 2.0 license][apache-2.0].

## MoveIt! quickstart for Panda research

Launch the joint trajectory controller

`roslaunch franka_control franka_control.launch robot_ip:=<ip>`

Launch MoveIt!

`roslaunch panda_moveit_config panda_moveit.launch`

Launch RViz

`roslaunch panda_moveit_config moveit_rviz.launch`

[apache-2.0]: https://www.apache.org/licenses/LICENSE-2.0.html
[fci-docs]: https://frankaemika.github.io/docs
[travis-status]: https://travis-ci.org/frankaemika/franka_ros.svg?branch=kinetic-devel
[travis]: https://travis-ci.org/frankaemika/franka_ros
