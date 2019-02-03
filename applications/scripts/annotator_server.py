#!/usr/bin/env python

import rospy

from map_annotator import Annotator
from map_annotator.msg import PoseNames
from map_annotator.msg import UserAction


def wait_for_time():
    """Wait for simulated time to begin.                          
    """
    while rospy.Time().now().to_sec() == 0:
        pass

class AnnotatorServer(object):
    def __init__(self):
        self._annotator = Annotator()
        self._pose_names_pub = rospy.Publisher("/map_annotator/pose_names",
                                               PoseNames, queue_size=10, latch=True)

    def __pub_pose_names__(self):
        pose_names = PoseNames()
        pose_names.names = self._annotator.get_positions()
        self._pose_names_pub.publish(pose_names)

    def handle_callback(self, user_action_msg):
        cmd = user_action_msg.command
        name = user_action_msg.name
        if cmd == UserAction.CREATE:
            self._annotator.save_position(name)
            self.__pub_pose_names__()
        elif cmd == UserAction.DELETE:
            self._annotator.delete_position(name)
            self.__pub_pose_names__()
        elif cmd == UserAction.GOTO:
            self._annotator.goto_position(name)
        

def main():
    rospy.init_node("annotator_server")
    wait_for_time()
    server = AnnotatorServer()
    rospy.Subscriber("/map_annotator/user_actions",
                     UserAction, server.handle_callback)
    rospy.spin()
 


if __name__ == '__main__':
  main()
