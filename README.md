# ROS integration for Franka Emika research robots

See the [Franka Control Interface (FCI) documentation][fci-docs] for more information.

## License

All packages of `franka_ros` are licensed under the [Apache 2.0 license][apache-2.0].

[apache-2.0]: https://www.apache.org/licenses/LICENSE-2.0.html
[fci-docs]: https://frankaemika.github.io/docs
******************************************************************************************

## What is added in this repository?

1. A task named (`cartesian_pickup_task`) in package (`franka_example_controllers`).
This task is a simple application of the franka_ros Cartesian controller, where the
robot will move to a certain (x,y,z) specified by the (.cpp) code and pick-up an
object with a certain width and move it to nother (x,y,z) location and release it.
The robot finally return back to his home position.

2. A task named (`cartesian_text_write_task`) in package (`franka_example_controllers`).
    This task is a simple application of the franka_ros Cartesian controller, where the
    robot will move to a certain (x,y,z) specified by the (.cpp) code and start writing
    a pre-specified three letters (in this code is  IAI). the robot then return to his
    home position.

 ******************************************************************************************
