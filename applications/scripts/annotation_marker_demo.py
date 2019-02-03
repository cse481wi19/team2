#!/usr/bin/env python
import map_annotator
from geometry_msgs.msg import Point
import rospy

def main():
    rospy.init_node('circle')
    rospy.sleep(0.5)
    circle_marker = map_annotator.CircleMarker()
    point = Point()
    circle_marker.create_draggable_marker(point)
    rospy.spin()

if __name__ == "__main__":
    main()