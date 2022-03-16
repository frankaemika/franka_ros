#!/usr/bin/env python
"""
Launches the controller spawner node after initial joint
positions of the robot are initialized. The command line args of are passed to the spawner node.
"""
import rospy
import roslaunch
import sys
from sensor_msgs.msg import JointState


def main():
    args = ' '.join(sys.argv[1:])
    package = 'controller_manager'
    node_type = 'spawner'
    rospy.init_node('delayed_controller_spawner')
    rospy.loginfo('Delay spawning of controller. Going to sleep...')
    while True:
        try:
            joint_state = rospy.wait_for_message('joint_states', JointState)  # type: JointState
            # check that the 4. joint ( valid range: [-0.0698,-3.0718]) is not zero anymore
            if joint_state.position[3] < -0.05:
                break
            rospy.sleep(0.1)
        except rospy.ROSInterruptException:
            rospy.logerr("ROSInterruptException")
            break

    rospy.loginfo('Waking up. Spawning controller...')
    node = roslaunch.core.Node(package, node_type, name=node_type, args=args)
    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()
    launch.launch(node)
    rospy.loginfo('Controller spawned')
    try:
        launch.spin()
    finally:
        launch.stop()


if __name__ == '__main__':
    main()
