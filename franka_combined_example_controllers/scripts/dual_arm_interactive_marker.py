<<<<<<< HEAD
#!/usr/bin/python

import rospy
import tf.transformations
import numpy as np
=======
#!/usr/bin/env python
""" This simple script creates an interactive marker for changing desired centering pose of
    two the dual_panda_cartesian_impedance_example_controller. It features also resetting the
    marker to current centering pose between the left and the right endeffector.
"""

import rospy
>>>>>>> multi_panda_beta
import argparse
from threading import Thread, Lock

from interactive_markers.interactive_marker_server import \
    InteractiveMarkerServer, InteractiveMarkerFeedback
from visualization_msgs.msg import InteractiveMarker, \
    InteractiveMarkerControl, Marker
<<<<<<< HEAD
from franka_msgs.msg import FrankaState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool 

mutex = Lock()
marker_pose = PoseStamped()
initial_pose_found = False
has_error = False
pose_pub = None
# [[min_x, max_x], [min_y, max_y], [min_z, max_z]]
position_limits = [[-0.6, 0.6], [-0.6, 0.6], [0.05, 0.9]]

def makeSphere(scale = 0.3):
=======
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

marker_pose = PoseStamped()

centering_frame_ready = False

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
>>>>>>> multi_panda_beta
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

<<<<<<< HEAD
def publisherCallback(msg, arm_id):
    marker_pose.header.frame_id = arm_id + "_link0"
    marker_pose.header.stamp = rospy.Time(0)
    pose_pub.publish(marker_pose)

def has_error_callback(msg):
    global has_error
    has_error = msg.data

def franka_state_callback(msg):
    initial_quaternion = \
        tf.transformations.quaternion_from_matrix(
            np.transpose(np.reshape(msg.O_T_EE,
                                    (4, 4))))
    initial_quaternion = initial_quaternion / np.linalg.norm(initial_quaternion)
    mutex.acquire()
    marker_pose.pose.orientation.x = initial_quaternion[0]
    marker_pose.pose.orientation.y = initial_quaternion[1]
    marker_pose.pose.orientation.z = initial_quaternion[2]
    marker_pose.pose.orientation.w = initial_quaternion[3]
    marker_pose.pose.position.x = msg.O_T_EE[12]
    marker_pose.pose.position.y = msg.O_T_EE[13]
    marker_pose.pose.position.z = msg.O_T_EE[14]
    mutex.release()
    global initial_pose_found
    initial_pose_found = True

def updateMarkerPoseBlocking(arm_id):
    state_sub = rospy.Subscriber(arm_id + "_state_controller/franka_states",
                                 FrankaState, franka_state_callback)
    # Get initial pose for the interactive marker
    global initial_pose_found
    initial_pose_found = False
    while not initial_pose_found:
        rospy.sleep(0.1)
    state_sub.unregister()

def processFeedback(feedback):
    if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
        mutex.acquire()
        marker_pose.pose.position.x = max([min([feedback.pose.position.x,
                                          position_limits[0][1]]),
                                          position_limits[0][0]])
        marker_pose.pose.position.y = max([min([feedback.pose.position.y,
                                          position_limits[1][1]]),
                                          position_limits[1][0]])
        marker_pose.pose.position.z = max([min([feedback.pose.position.z,
                                          position_limits[2][1]]),
                                          position_limits[2][0]])
        marker_pose.pose.orientation = feedback.pose.orientation
        mutex.release()
=======

def publish_target_pose():
    """
    This function publishes desired centering pose which the controller will subscribe to.
    :return: None
    """
    marker_pose.header.stamp = rospy.Time(0)
    pose_pub.publish(marker_pose)


def left_has_error_callback(msg):
    """
    This callback function set `has_error` variable to True if the left arm is having an error.
    :param msg: error msg data
    :return:  None
    """
    global has_error, left_has_error, right_has_error
    left_has_error = msg.data
    has_error = left_has_error or right_has_error


def right_has_error_callback(msg):
    """
    This callback function set `has_error` variable to True if the right arm is having an error.
    :param msg: error msg data
    :return:  None
    """
    global has_error, left_has_error, right_has_error
    right_has_error = msg.data
    has_error = left_has_error or right_has_error


def centering_pose_callback(msg):
    """
    This callback function sets the marker pose to the current centering pose from a subscribed topic.
    :return: None
    """
    global centering_frame_ready
    global marker_pose

    marker_pose = msg
    centering_frame_ready = True


def reset_marker_pose_blocking():
    """
    This function resets the marker pose to current "middle pose" of left and right arm EEs.
    :return: None
    """

    global centering_frame_ready
    global marker_pose

    centering_frame_ready = False

    centering_frame_pose_sub = rospy.Subscriber(
        "dual_arm_cartesian_impedance_example_controller/centering_frame",
        PoseStamped, centering_pose_callback)

    # Get initial pose for the interactive marker
    while not centering_frame_ready:
        rospy.sleep(0.1)

    centering_frame_pose_sub.unregister()


def process_feedback(feedback):
    """
    This callback function clips the marker_pose inside a predefined box to prevent misuse of the marker.
    :param feedback: feedback data of interactive marker
    :return: None
    """
    global marker_pose
    if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
        marker_pose.pose.position.x = feedback.pose.position.x
        marker_pose.pose.position.y = feedback.pose.position.y
        marker_pose.pose.position.z = feedback.pose.position.z
        marker_pose.pose.orientation = feedback.pose.orientation
>>>>>>> multi_panda_beta
    server.applyChanges()


if __name__ == "__main__":
    rospy.init_node("target_pose_node")

<<<<<<< HEAD
    parser = argparse.ArgumentParser("dual_arm_interactive_marker.py")
    parser.add_argument("--arm_id", help="The id of the arm.", required=True)
    parser.add_argument('args', nargs=argparse.REMAINDER)
    args = parser.parse_args()

    listener = tf.TransformListener()

    arm_id = args.arm_id

    error_sub = rospy.Subscriber(arm_id + "/has_error",
                                 Bool, has_error_callback)
    updateMarkerPoseBlocking(arm_id)

    pose_pub = rospy.Publisher(
      "dual_arm_cartesian_impedance_example_controller/" + arm_id + "/target_pose", PoseStamped, queue_size=1)
    server = InteractiveMarkerServer("target_pose_marker_" + arm_id)
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = arm_id + "_link0"
    int_marker.scale = 0.3
    int_marker.name = arm_id + "_target_pose"
    int_marker.description = ("Target Pose\nBE CAREFUL! "
                              "If you move the \ntarget "
                              "pose the robot will follow it\n"
                              "so be aware of potential collisions")
    int_marker.pose = marker_pose.pose
    # run pose publisher
    rospy.Timer(rospy.Duration(0.005),
                lambda msg: publisherCallback(msg, arm_id))
=======
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
    left_error_sub = rospy.Subscriber(left_arm_id + "/has_error", Bool,
                                      left_has_error_callback)
    right_error_sub = rospy.Subscriber(right_arm_id + "/has_error", Bool,
                                       right_has_error_callback)

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
>>>>>>> multi_panda_beta

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
<<<<<<< HEAD
    control.markers.append(makeSphere())
    control.interaction_mode = InteractiveMarkerControl.MOVE_3D
    int_marker.controls.append(control)

    server.insert(int_marker, processFeedback)

    server.applyChanges()

    while not rospy.is_shutdown():
        if has_error:
            updateMarkerPoseBlocking(arm_id)
            server.setPose(arm_id + "_target_pose", marker_pose.pose)
=======
    control.markers.append(make_sphere())
    control.interaction_mode = InteractiveMarkerControl.MOVE_3D
    int_marker.controls.append(control)

    server.insert(int_marker, process_feedback)
    server.applyChanges()

    global marker_pose

    # main loop
    while not rospy.is_shutdown():
        publish_target_pose()
        if has_error:
            reset_marker_pose_blocking()
            publish_target_pose()
            server.setPose("centering_frame_pose", marker_pose.pose)
>>>>>>> multi_panda_beta
            server.applyChanges()
            rospy.sleep(0.5)
        rospy.sleep(0.1)
