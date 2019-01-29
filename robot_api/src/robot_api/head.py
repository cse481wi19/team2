#!/usr/bin/env python

# TODO: import ?????????
# TODO: import ???????_msgs.msg
# TODO: import ??????????_msgs.msg
import math

import actionlib
import rospy
from trajectory_msgs.msg import JointTrajectoryPoint
from trajectory_msgs.msg import JointTrajectory

from control_msgs.msg import JointTrajectoryAction
from control_msgs.msg import JointTrajectoryGoal

from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryGoal

from control_msgs.msg import PointHeadGoal
from control_msgs.msg import PointHeadAction

from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Point

LOOK_AT_ACTION_NAME = '/head_controller/point_head/'  # TODO: Get the name of the look-at action
PAN_TILT_ACTION_NAME = '/head_controller/follow_joint_trajectory/'  # TODO: Get the name of the pan/tilt action
PAN_JOINT = 'head_pan_joint'  # TODO: Get the name of the head pan joint
TILT_JOINT = 'head_tilt_joint'  # TODO: Get the name of the head tilt joint
PAN_TILT_TIME = 2.5  # How many seconds it should take to move the head.


class Head(object):
    """Head controls the Fetch's head.

    It provides two interfaces:
        head.look_at(frame_id, x, y, z)
        head.pan_tilt(pan, tilt) # In radians

    For example:
        head = robot_api.Head()
        head.look_at('base_link', 1, 0, 0.3)
        head.pan_tilt(0, math.pi/4)
    """
    MIN_PAN = -math.pi/2   # TODO: Minimum pan angle, in radians. -90 degrees
    MAX_PAN = math.pi/2    # TODO: Maximum pan angle, in radians. +90 degrees
    MIN_TILT = -math.pi/4  # TODO: Minimum tilt angle, in radians.
    MAX_TILT =  math.pi/2  # TODO: Maximum tilt angle, in radians.

    def __init__(self):
        # TODO: Create actionlib clients
        self._look_client = actionlib.SimpleActionClient(LOOK_AT_ACTION_NAME, PointHeadAction)
        self._pan_tilt_client = actionlib.SimpleActionClient(PAN_TILT_ACTION_NAME, FollowJointTrajectoryAction)
        # TODO: Wait for both servers

        self._look_client.wait_for_server()
        self._pan_tilt_client.wait_for_server()
        pass

    def look_at(self, frame_id, x, y, z):
        """Moves the head to look at a point in space.

        Args:
            frame_id: The name of the frame in which x, y, and z are specified.
            x: The x value of the point to look at.
            y: The y value of the point to look at.
            z: The z value of the point to look at.
        """
        # TODO: Create goal
        goal = PointHeadGoal()

        # t = JointTrajectory()
        # t.header.frame_id = frame_id

        pointStamped = PointStamped()
        point = Point()
        point.x = x
        point.y = y
        point.z = z
        # point.positions.append(y)
        # point.positions.append(z)
        pointStamped.point = point
        # goal.target.append(point)
        goal.target = pointStamped

        goal.target.header.frame_id = frame_id

        # t.points.append(point)

        # t.joint_names.append(PAN_JOINT)
        # t.joint_names.append(TILT_JOINT)

        # TODO: Fill out the goal (we recommend setting min_duration to 1 second)
        goal.min_duration = rospy.Duration(secs=1,nsecs=0)

        # TODO: Send the goal
        # goal.trajectory = t
        self._look_client.send_goal(goal)

        # TODO: Wait for result
        self._look_client.wait_for_result()
        # rospy.logerr('Not implemented.')

    def pan_tilt(self, pan, tilt):
        """Moves the head by setting pan/tilt angles.

              Args:
            pan: The pan angle, in radians. A positive value is clockwise.
            tilt: The tilt angle, in radians. A positive value is downwards.
        """
        # TODO: Check that the pan/tilt angles are within joint limits
        if not ((pan >= Head.MIN_PAN and pan <= Head.MAX_PAN) and (tilt >= Head.MIN_TILT and tilt <= Head.MAX_TILT)):
            # Not within join limits
            return

        # TODO: Create a trajectory point
        point = JointTrajectoryPoint()

        # TODO: Set positions of the two joints in the trajectory point
        point.positions.append(pan)
        point.positions.append(tilt)

        # TODO: Set time of the trajectory point
        point.time_from_start = rospy.Duration(secs=2.5, nsecs=0)

        # TODO: Create goal
        goal = FollowJointTrajectoryGoal()

        traj = JointTrajectory()
        # TODO: Add joint names to the list
        traj.joint_names.append(PAN_JOINT)
        traj.joint_names.append(TILT_JOINT)

        # TODO: Add trajectory point created above to trajectory
        traj.points.append(point)

        goal.trajectory = traj
        # TODO: Send the goal
        self._pan_tilt_client.send_goal(goal)

        # TODO: Wait for result
        self._pan_tilt_client.wait_for_result()

        # rospy.logerr('Not implemented.')