#!/usr/bin/python

import rospy
import tf.transformations
import numpy as np
import argparse
from threading import Thread, Lock

from interactive_markers.interactive_marker_server import \
    InteractiveMarkerServer, InteractiveMarkerFeedback
from visualization_msgs.msg import InteractiveMarker, \
    InteractiveMarkerControl, Marker
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
    server.applyChanges()


if __name__ == "__main__":
    rospy.init_node("target_pose_node")

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
    control.markers.append(makeSphere())
    control.interaction_mode = InteractiveMarkerControl.MOVE_3D
    int_marker.controls.append(control)

    server.insert(int_marker, processFeedback)

    server.applyChanges()

    while not rospy.is_shutdown():
        if has_error:
            updateMarkerPoseBlocking(arm_id)
            server.setPose(arm_id + "_target_pose", marker_pose.pose)
            server.applyChanges()
            rospy.sleep(0.5)
        rospy.sleep(0.1)
