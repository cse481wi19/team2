#! /usr/bin/env python

import robot_api
import rospy


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


def main():
    rospy.init_node('<insert random name here>')
    wait_for_time()
    argv = rospy.myargv()
    

if __name__ == '__main__':
    main()
