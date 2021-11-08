#!/usr/bin/env python
""" This simple script creates an interactive marker for changing desired centering pose of
    two the dual_panda_cartesian_impedance_example_controller. It features also resetting the
    marker to current centering pose between the left and the right endeffector.
"""

import rospy
import argparse

from interactive_markers.interactive_marker_server import \
    InteractiveMarkerServer, InteractiveMarkerFeedback
from visualization_msgs.msg import InteractiveMarker, \
    InteractiveMarkerControl, Marker
from franka_msgs.msg import FrankaState
from geometry_msgs.msg import PoseStamped

marker_pose = PoseStamped()

has_error = False
left_has_error = False
right_has_error = False

pose_pub = None


def make_sphere(scale=0.3):
    """
    This function returns sphere marker for 3D translational movements.
    :param scale: scales the size of the sphere
    :return: sphere marker
    """
    marker = Marker()
    marker.type = Marker.SPHERE
    marker.scale.x = scale * 0.45
    marker.scale.y = scale * 0.45
    marker.scale.z = scale * 0.45
    marker.color.r = 0.5
    marker.color.g = 0.5
    marker.color.b = 0.5
    marker.color.a = 1.0
    return marker


def publish_target_pose():
    """
    This function publishes desired centering pose which the controller will subscribe to.
    :return: None
    """
    marker_pose.header.stamp = rospy.Time(0)
    pose_pub.publish(marker_pose)


def left_franka_state_callback(msg):
    """
    This callback function set `has_error` variable to True if the left arm is having an error.
    :param msg: FrankaState msg data
    :return:  None
    """
    global has_error, left_has_error
    if msg.robot_mode == FrankaState.ROBOT_MODE_MOVE:
        left_has_error = False
    else:
        left_has_error = True
    has_error = left_has_error or right_has_error


def right_franka_state_callback(msg):
    """
    This callback function set `has_error` variable to True if the right arm is having an error.
    :param msg: FrankaState msg data
    :return:  None
    """
    global has_error, right_has_error
    if msg.robot_mode == FrankaState.ROBOT_MODE_MOVE:
        right_has_error = False
    else:
        right_has_error = True
    has_error = left_has_error or right_has_error


def reset_marker_pose_blocking():
    """
    This function resets the marker pose to current "middle pose" of left and right arm EEs.
    :return: None
    """

    global marker_pose
    marker_pose = rospy.wait_for_message(
        "dual_arm_cartesian_impedance_example_controller/centering_frame", PoseStamped)


def process_feedback(feedback):
    """
    This callback function clips the marker_pose inside a predefined box to prevent misuse of the
    marker.
    :param feedback: feedback data of interactive marker
    :return: None
    """
    global marker_pose
    if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
        marker_pose.pose.position.x = feedback.pose.position.x
        marker_pose.pose.position.y = feedback.pose.position.y
        marker_pose.pose.position.z = feedback.pose.position.z
        marker_pose.pose.orientation = feedback.pose.orientation
    server.applyChanges()


if __name__ == "__main__":
    rospy.init_node("target_pose_node")

    parser = argparse.ArgumentParser("dual_panda_interactive_marker.py")
    parser.add_argument("--left_arm_id",
                        help="The id of the left arm.",
                        required=True)
    parser.add_argument("--right_arm_id",
                        help="The id of the right arm.",
                        required=True)
    parser.add_argument('args', nargs=argparse.REMAINDER)
    args = parser.parse_args()

    # Arm IDs of left and right arms
    left_arm_id = args.left_arm_id
    right_arm_id = args.right_arm_id

    # Initialize subscribers for error states of the arms
    left_state_sub = rospy.Subscriber(left_arm_id + "_state_controller/franka_states",
                                      FrankaState, left_franka_state_callback)

    right_state_sub = rospy.Subscriber(right_arm_id + "_state_controller/franka_states",
                                       FrankaState, right_franka_state_callback)

    # Set marker pose to be the current "middle pose" of both EEs
    reset_marker_pose_blocking()

    # Initialize publisher for publishing desired centering pose
    pose_pub = rospy.Publisher(
        "dual_arm_cartesian_impedance_example_controller/centering_frame_target_pose",
        PoseStamped,
        queue_size=1)

    # Interactive marker settings
    server = InteractiveMarkerServer("target_pose_marker")
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = marker_pose.header.frame_id
    int_marker.scale = 0.3
    int_marker.name = "centering_frame_pose"
    int_marker.description = ("Target centering pose\n"
                              "BE CAREFUL! \n"
                              "If you move the target marker\n"
                              "both robots will follow it \n"
                              "as the center between the two\n"
                              "endeffectors. Be aware of\n"
                              "potential collisions!")
    int_marker.pose = marker_pose.pose

    # insert a box
    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 1
    control.orientation.y = 0
    control.orientation.z = 0
    control.name = "rotate_x"
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    int_marker.controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 1
    control.orientation.y = 0
    control.orientation.z = 0
    control.name = "move_x"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    int_marker.controls.append(control)
    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 1
    control.orientation.z = 0
    control.name = "rotate_y"
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    int_marker.controls.append(control)
    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 1
    control.orientation.z = 0
    control.name = "move_y"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    int_marker.controls.append(control)
    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 0
    control.orientation.z = 1
    control.name = "rotate_z"
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    int_marker.controls.append(control)
    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 0
    control.orientation.z = 1
    control.name = "move_z"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    int_marker.controls.append(control)
    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 1
    control.orientation.y = 1
    control.orientation.z = 1
    control.name = "move_3D"
    control.always_visible = True
    control.markers.append(make_sphere())
    control.interaction_mode = InteractiveMarkerControl.MOVE_3D
    int_marker.controls.append(control)

    server.insert(int_marker, process_feedback)
    server.applyChanges()

    # main loop
    while not rospy.is_shutdown():
        publish_target_pose()
        if has_error:
            reset_marker_pose_blocking()
            publish_target_pose()
            server.setPose("centering_frame_pose", marker_pose.pose)
            server.applyChanges()
            rospy.sleep(0.5)
        rospy.sleep(0.1)
