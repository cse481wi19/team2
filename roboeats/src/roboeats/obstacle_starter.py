#!/usr/bin/env python

import rospy

from map_annotator import Annotator
from server import RoboEatsServer
from food_item import FoodItem

def wait_for_time():                                              
    """Wait for simulated time to begin.                          
    """                                                           
    while rospy.Time().now().to_sec() == 0:                       
        pass

def main():
    rospy.init_node("obstacle_starter")
    wait_for_time()

    server = RoboEatsServer()

    server.start_obstacles_2()

    rospy.spin()


if __name__ == '__main__':
  main()
