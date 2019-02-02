#!/usr/bin/env python

import rospy

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA

def wait_for_time():                                              
    """Wait for simulated time to begin.                          
    """                                                           
    while rospy.Time().now().to_sec() == 0:                       
        pass

def print_intro():
    print("Welcome to the map annotator!")
    print("Commands:")
    print("list: List saved poses.")
    print("save <name>: Save the robot's current pose as <name>. Overwrites if <name> already exists.")
    print("delete <name>: Delete the pose given by <name>.")
    print("goto <name>: Sends the robot to the pose given by <name>.")
    print("help: Show this list of commands")

def main():
    running = True
    while running:
        user_input = input()
        print user_input
    # rospy.init_node('annotator_node')
    # wait_for_time()
    # marker_publisher = rospy.Publisher('visualization_marker', Marker, queue_size=10)
    # rospy.sleep(0.5)
    # show_text_in_rviz(marker_publisher, "RoboEats is cool.")

if __name__ == '__main__':
  main()