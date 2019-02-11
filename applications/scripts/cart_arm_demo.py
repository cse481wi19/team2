#! /usr/bin/env python

import math
import robot_api
import rospy

from geometry_msgs.msg import Point, Pose, Quaternion, PoseStamped
                         
def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass
        
def main():
    # ... init ...
    rospy.init_node('arm_moveit_demo')
    wait_for_time()
    argv = rospy.myargv()

    arm = robot_api.Arm()
    def shutdown():
        arm.cancel_all_goals()
    rospy.on_shutdown(shutdown)

    pose1 = Pose(Point(0.042, 0.384, 1.826), Quaternion(0.173, -0.693, -0.242, 0.657))
    pose2 = Pose(Point(0.047, 0.545, 1.822), Quaternion(-0.274, -0.701, 0.173, 0.635))
    ps1 = PoseStamped()
    ps1.header.frame_id = 'base_link'
    ps1.pose = pose1
    ps2 = PoseStamped()
    ps2.header.frame_id = 'base_link'
    ps2.pose = pose2
    gripper_poses = [ps2, ps1]
    i = 0
    while True:
        pose = gripper_poses[i % len(gripper_poses)]
        print("moveit to pose: ", pose)
        error = arm.move_to_pose(pose)
        rospy.sleep(1)
        if error is not None:
            rospy.logerr(error)
        i += 1

    # Move the arm


if __name__ == '__main__':
    main()