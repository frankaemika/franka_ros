# Schmalz cobot vacuum gripper for FRANKA EMIKA Panda arm
This package is written to allow for the control of the schmalz cobot  vacuum gripper designed for the Franka Emika Panda through ROS.

It is in essence a copy of the [franka_gripper]() package, I simply changed some small things to point the methods to the vacuum gripper class already present in Libfranka (0.8.0).

## Installation
If libfranka and franka ros are not yet installed, do that first, following the steps as given by Franka Emika (add link)

Once that is done, clone this package into your workspace
```bash
git clone (add link)
```

Then, build the package

```bash
catkin build -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=path/to/libfranka/build franka_vacuum_gripper
```

## Usage
Make sure the Schmalz cobot vacuum gripper is installed on the Franka Emika Panda arm as instructed and that the vacuum gripper is selected as being the current end-effector in the franka desk application.

Launch the franka_vacuum_gripper_node
```bash
cd catkin_ws
source devel/setup.bash
roslaunch franka_vacuum_gripper franka_vacuum_gripper.launch robot_ip:=[your robot_ip]
```

This launches three different action services:
1. vacuum  
in: vacuum [per 10 mbar]
in: timeout [ms]
out: success [bool]
out: error [string]

This action will cause the gripper to apply a vacuum. As soon as the vacuum level is reached, the action succeeds.

2. drop off
in: timeout [ms]
out: success [bool]
out: error [string]

This action will cause the gripper to stop applying the vacuum and, additionally, release the vacuum so that the grasped object drops.

3. stop
out: success [bool]
out: error [string]

This action will stop the gripper to apply a vacuum but as far as I understand it does not release the vacuum, causing the object to stay attached to the gripper.



