#!/usr/bin/env python

# TODO: import ?????????
# TODO: import ???????_msgs.msg
# TODO: import ??????????_msgs.msg

import actionlib
import rospy
from trajectory_msgs.msg import JointTrajectoryPoint
from trajectory_msgs.msg import JointTrajectory

from control_msgs.msg import JointTrajectoryAction
from control_msgs.msg import JointTrajectoryGoal

from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryGoal



# TODO: ACTION_NAME = ???
ACTION_NAME = "/torso_controller/follow_joint_trajectory/"
# ACTION_NAME = "/joint_trajectory_controller/joint_trajectory/"
# TODO: JOINT_NAME = ???
JOINT_NAME = "torso_lift_joint"
TIME_FROM_START = 5  # How many seconds it should take to set the torso height.


class Torso(object):
    """Torso controls the robot's torso height.
    """
    MIN_HEIGHT = 0.0
    MAX_HEIGHT = 0.4

    def __init__(self):
        # TODO: Create actionlib client
        self._client = actionlib.SimpleActionClient(ACTION_NAME, FollowJointTrajectoryAction)
        # self._client = actionlib.SimpleActionClient(ACTION_NAME, JointTrajectoryAction)
        # TODO: Wait for server
        self._client.wait_for_server()
        pass

    def set_height(self, height):
        """Sets the torso height.

        This will always take ~5 seconds to execute.

        Args:
            height: The height, in meters, to set the torso to. Values range
                from Torso.MIN_HEIGHT (0.0) to Torso.MAX_HEIGHT(0.4).
        """
        # TODO: Check that the height is between MIN_HEIGHT and MAX_HEIGHT.
        if (not (height >= Torso.MIN_HEIGHT and height <= Torso.MAX_HEIGHT)):
            # Don't do anything if height is outside of range
            return

        # TODO: Create a trajectory point
        point = JointTrajectoryPoint()
        # TODO: Set position of trajectory point
        point.positions.append(height)
        # TODO: Set time of trajectory point
        point.time_from_start = rospy.Duration(secs=5, nsecs=0)

        # TODO: Create goal
        goal = FollowJointTrajectoryGoal()

        traj = JointTrajectory()
        # TODO: Add joint name to list
        traj.joint_names.append(JOINT_NAME)
        # TODO: Add the trajectory point created above to trajectory
        traj.points.append(point)

        goal.trajectory = traj
        # TODO: Send goal
        self._client.send_goal(goal)
        # TODO: Wait for result
        self._client.wait_for_result()
