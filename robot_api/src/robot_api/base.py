#! /usr/bin/env python

# TODO: import ????????_msgs.msg
import rospy
from geometry_msgs.msg import Twist, Vector3


class Base(object):
    """Base controls the mobile base portion of the Fetch robot.

    Sample usage:
        base = fetch_api.Base()
        while CONDITION:
            base.move(0.2, 0)
        base.stop()
    """

    def __init__(self):
        # TODO: Create publisher
        self._pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        pass

    def move(self, linear_speed, angular_speed):
        """Moves the base instantaneously at given linear and angular speeds.

        "Instantaneously" means that this method must be called continuously in
        a loop for the robot to move.

        Args:
            linear_speed: The forward/backward speed, in meters/second. A
                positive value means the robot should move forward.
            angular_speed: The rotation speed, in radians/second. A positive
                value means the robot should rotate clockwise.
        """
        # TODO: Create Twist msg
        twist = Twist()
        # TODO: Fill out msg
        linear_v = Vector3()
        linear_v.x = linear_speed
        linear_v.y = 0
        linear_v.z = 0

        angular_v = Vector3()
        angular_v.x = 0
        angular_v.y = 0
        angular_v.z = angular_speed

        twist.linear = linear_v
        twist.angular = angular_v
        # TODO: Publish msg
        self._pub.publish(twist)
        # rospy.logerr('Not implemented.')

    def stop(self):
        """Stops the mobile base from moving.
        """
        # TODO: Publish 0 velocity
        twist = Twist()
        twist.linear = 0
        twist.angular = 0
        self._pub.publish(twist)
        # rospy.logerr('Not implemented.')
