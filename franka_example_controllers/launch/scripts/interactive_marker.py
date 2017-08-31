#!/usr/bin/python

import rospy
import numpy
import tf

from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *
from geometry_msgs.msg import Point
from geometry_msgs.msg import Wrench
from tf.broadcaster import TransformBroadcaster

marker_pose = geometry_msgs.msg.PoseStamped()
pose_pub = None

def publisherCallback(msg):
    marker_pose.header.frame_id = "franka_emika_link0"
    marker_pose.header.stamp = rospy.Time(0)
    pose_pub.publish(marker_pose)

def processFeedback(feedback):
    if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
        marker_pose.pose = feedback.pose
    server.applyChanges()

if __name__ == "__main__":
    rospy.init_node("equilibrium_pose_node")
    listener = tf.TransformListener()
    pose_pub = rospy.Publisher("/equilibrium_pose", geometry_msgs.msg.PoseStamped, queue_size=4)

    # get initial pose through TF
    try:
        listener.waitForTransform("/franka_emika_link0", "/franka_emika_EE", rospy.Time(0), rospy.Duration(10.0))
        (initial_translation,initial_quaternion) = listener.lookupTransform("/franka_emika_link0", "/franka_emika_EE", rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rospy.loginfo("Coudn't find transforms franka_emika_EE transform!")

    server = InteractiveMarkerServer("equilibrium_pose_marker")
    # do I need this menu???
    menu_handler = MenuHandler()
    pf_wrap = lambda fb: processFeedback(fb)

    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "franka_emika_link0"
    int_marker.pose.position.x = initial_translation[0]
    int_marker.pose.position.y = initial_translation[1]
    int_marker.pose.position.z = initial_translation[2]
    int_marker.pose.orientation.x = initial_quaternion[0]
    int_marker.pose.orientation.y = initial_quaternion[1]
    int_marker.pose.orientation.z = initial_quaternion[2]
    int_marker.pose.orientation.w = initial_quaternion[3]
    int_marker.scale = 0.3
    int_marker.name = "equilibrium_pose"
    int_marker.description = "Equilibrium pose"

    # set initial message for the topic
    marker_pose.pose = int_marker.pose

    # Publisher for the topic
    rospy.Timer(rospy.Duration(0.002), publisherCallback)

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
    server.insert(int_marker, pf_wrap)

    server.applyChanges()

    rospy.spin()