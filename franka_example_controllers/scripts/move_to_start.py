#!/usr/bin/env python

import sys
import rospy as ros

ros.init_node('move_to_start')

from actionlib import SimpleActionClient
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, FollowJointTrajectoryResult


action = ros.resolve_name('~follow_joint_trajectory')
client = SimpleActionClient(action, FollowJointTrajectoryAction)
ros.loginfo("move_to_start: Waiting for '" + action + "' action to come up")
client.wait_for_server()

param = ros.resolve_name('~joint_pose')
pose = ros.get_param(param, None)
if pose is None:
    ros.logerr('move_to_start: Could not find required parameter "' + param + '"')
    sys.exit(1)

point = JointTrajectoryPoint()
point.time_from_start = ros.Duration.from_sec(ros.get_param('~duration', 5.))
goal = FollowJointTrajectoryGoal()

goal.trajectory.joint_names, point.positions = [list(x) for x in zip(*pose.items())]

goal.trajectory.points.append(point)
goal.goal_time_tolerance = ros.Duration.from_sec(0.5)

ros.loginfo('Sending trajectory Goal to move into initial config')
client.send_goal_and_wait(goal)

result = client.get_result()
if result.error_code != FollowJointTrajectoryResult.SUCCESSFUL:
    ros.logerr('move_to_start: Movement was not successful: ' + {
        FollowJointTrajectoryResult.INVALID_GOAL: "The joint pose you want to move to is invalid (e.g. unreachable, singularity...). Is the 'joint_pose' reachable?",
        FollowJointTrajectoryResult.INVALID_JOINTS: "The joint pose you specified is for different joints than the joint trajectory controller is claiming. Does you 'joint_pose' include all 7 joints of the robot?",
        FollowJointTrajectoryResult.PATH_TOLERANCE_VIOLATED: "During the motion the robot deviated from the planned path too much. Is something blocking the robot?",
        FollowJointTrajectoryResult.GOAL_TOLERANCE_VIOLATED: "After the motion the robot deviated from the desired goal pose too much. Probably the robot didn't reach the joint_pose properly",
    }[result.error_code])

else:
    ros.loginfo('move_to_start: Successfully moved into start pose')
