#! /usr/bin/env python

import math
import copy
import tf.transformations as tft
import numpy as np
import rospy
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry


def distance_fn(curr_loc, last_location):
    return ((curr_loc.x - last_location.x)**2 + 
            (curr_loc.y - last_location.y)**2 + 
            (curr_loc.z - last_location.z)**2)**0.5

def quart_to_yaw(q):
    m = tft.quaternion_matrix([q.x, q.y, q.z, q.w])
    x = m[0, 0]
    y = m[1, 0]
    theta_rads = math.atan2(y, x)
    if theta_rads < 0:
        theta_rads = 2 * math.pi + theta_rads
    return theta_rads

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
        self._odom_sub = rospy.Subscriber('odom', Odometry, callback=self._odom_callback)

    def _odom_callback(self, msg):
        # TODO: do something
        pass

    def go_forward(self, distance, speed=0.1):
        """Moves the robot a certain distance.

        It's recommended that the robot move slowly. If the robot moves too
        quickly, it may overshoot the target. Note also that this method does
        not know if the robot's path is perturbed (e.g., by teleop). It stops
        once the distance traveled is equal to the given distance or more.

        Args:
            distance: The distance, in meters, to move. A positive value
                means forward, negative means backward.
            speed: The speed to travel, in meters/second.
        """
        # TODO: rospy.sleep until the base has received at least one message on /odom
        odom_msg = rospy.wait_for_message("/odom", Odometry)
        # TODO: record start position, use Python's copy.deepcopy
        start = copy.deepcopy(odom_msg)
        rate = rospy.Rate(10)
        # TODO: CONDITION should check if the robot has traveled the desired distance
        # TODO: Be sure to handle the case where the distance is negative!
        TOLERANCE = 0.01
        dir = -1 if distance < 0 else 1
        curr_dist = 0
        while abs(distance) - curr_dist >= TOLERANCE:
            print("curr_dist:", curr_dist)
            # TODO: you will probably need to do some math in this loop to check the CONDITION
            self.move(dir * speed, 0)
            rate.sleep()
            
            curr_pos = rospy.wait_for_message("/odom", Odometry)
            curr = copy.deepcopy(curr_pos)
            curr_dist = distance_fn(start.pose.pose.position, curr.pose.pose.position)
        print("final_dist:", curr_dist)

    def turn(self, angular_distance, speed=0.5):
        """Rotates the robot a certain angle.

        Args:
            angular_distance: The angle, in radians, to rotate. A positive
                value rotates counter-clockwise.
            speed: The angular speed to rotate, in radians/second.
        """
        print("angular_distance given: ", angular_distance)
        # TODO: rospy.sleep until the base has received at least one message on /odom
        odom_msg = rospy.wait_for_message("/odom", Odometry)
        # TODO: record start position, use Python's copy.deepcopy
        start = copy.deepcopy(odom_msg)
        start_yaw = quart_to_yaw(start.pose.pose.orientation)
        # TODO: What will you do if angular_distance is greater than 2*pi or less than -2*pi?
        rate = rospy.Rate(10)
        # TODO: CONDITION should check if the robot has rotated the desired amount
        # TODO: Be sure to handle the case where the desired amount is negative!
        direction = -1 if angular_distance < 0 else 1
        # angular_distance = angular_distance % (2 * math.pi)
        ang_rotated = 0
        print("start_yaw: ", start_yaw, "angular_distance", angular_distance)
        prev_yaw = start_yaw
        while abs(angular_distance) - ang_rotated > 0:
            # TODO: you will probably need to do some math in this loop to check the CONDITION
            self.move(0, direction * speed)
            rate.sleep()

            curr_pos = rospy.wait_for_message("/odom", Odometry)
            curr = copy.deepcopy(curr_pos)
            curr_yaw = quart_to_yaw(curr.pose.pose.orientation)
            if direction == 1 and prev_yaw > curr_yaw:
                ang_rotated += curr_yaw - prev_yaw + 2 * math.pi
            elif direction == -1 and prev_yaw < curr_yaw:
                ang_rotated += prev_yaw - curr_yaw + 2 * math.pi
            else:
                ang_rotated += abs(abs(curr_yaw) - prev_yaw)
            print("ang_rotated: ", ang_rotated, "direction: ", direction, "curr_yaw: ", curr_yaw)
            prev_yaw = curr_yaw

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
