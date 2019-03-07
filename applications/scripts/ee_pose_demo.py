#! /usr/bin/env python


import rospy
import robot_api
import tf
import itertools
from moveit_python import PlanningSceneInterface
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import OrientationConstraint


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


def main():
    rospy.init_node("tf_demo")
    wait_for_time()
    argv = rospy.myargv()

    listener = tf.TransformListener()

    # We need to wait for 1 second fo the listener to know about the base_link.
    rospy.sleep(1)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        try:
            (trans, rot) = listener.lookupTransform(
                "base_link", "gripper_link", rospy.Time(0))
            # Rounding numbers
            for i, t in enumerate(trans):
                trans[i] = round(t, 5)
            for i, r in enumerate(rot):
                rot[i] = round(r, 5)
            rospy.loginfo(str(trans) + " " +  str(rot))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr(e)
            continue

        rate.sleep()

    return


if __name__ == '__main__':
    main()
