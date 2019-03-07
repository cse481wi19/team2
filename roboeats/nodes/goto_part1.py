#!/usr/bin/env python

import rospy

from map_annotator import Annotator

def wait_for_time():                                              
    """Wait for simulated time to begin.                          
    """                                                           
    while rospy.Time().now().to_sec() == 0:                       
        pass

def main():
    rospy.init_node("goto_part1")
    wait_for_time()
    annotator = Annotator()

    sleep_time = 1.5

    annotator.goto_position("start_location")
    rospy.sleep(sleep_time)
    annotator.goto_position("microwave_location_1")
    rospy.sleep(sleep_time)
    annotator.goto_position("microwave_location_2")

if __name__ == '__main__':
  main()
