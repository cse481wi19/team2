#! /usr/bin/env python


import rospy
import robot_api
import tf
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
    rospy.sleep(1)
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans, rot) = listener.lookupTransform(
                "l_gripper_finger_link", "r_gripper_finger_link", rospy.Time(0))
            rospy.loginfo(trans + rot)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr("got error")
            continue

        # rospy.loginfo(trans)
        # rospy.loginfo(rot)
        rate.sleep()
    
    return


if __name__ == '__main__':
    main()
