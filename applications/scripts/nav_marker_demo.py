#!/usr/bin/env python

from geometry_msgs.msg import Twist, Quaternion, Pose, Point, Vector3
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from std_msgs.msg import Header, ColorRGBA
import rospy

DIFFERENCE = 0.01

class NavPath(object):
    def __init__(self):
        self._marker_publisher = rospy.Publisher('visualization_marker', Marker, queue_size=10)
        self._path = []
            
    def callback(self, msg):
        rospy.loginfo(msg)
        curr_location = msg.pose.pose.position
        if len(self._path) == 0:
            self._path.append(curr_location)
        else:
            last_location = self._path[-1]
            if  distance(curr_location, last_location) > DIFFERENCE:
                self._path.append(curr_location)
        size = 0.05
        marker = Marker(
                type=Marker.LINE_STRIP,
                id=0,
                lifetime=rospy.Duration(10),
                points=self._path,
                # pose=Pose(self._path[0], Quaternion(0, 0, 0, 1)),
                scale=Vector3(size, 0, 0),
                header=Header(frame_id='odom', stamp=rospy.Time.now()),
                color=ColorRGBA(0.0, 1.0, 0.0, 0.8))
        self._marker_publisher.publish(marker)
        
                


def distance(curr_loc, last_location):
    return ((curr_loc.x - last_location.x)**2 + 
            (curr_loc.y - last_location.y)**2 + 
            (curr_loc.z - last_location.z)**2)**0.5
def main():
    rospy.init_node('my_node')
    rospy.sleep(0.5)
    print("NOOOOOOOOOOOOOOOO")
    nav_path = NavPath()
    rospy.Subscriber('odom', Odometry, nav_path.callback)
    rospy.spin()
    
if __name__ == '__main__':
  main()