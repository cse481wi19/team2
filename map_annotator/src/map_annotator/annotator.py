#!/usr/bin/env python                                                                                  
                                                                                                       
import rospy     
import pickle
import actionlib
import os

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseGoal, MoveBaseAction

class Annotator(object):                                

    def __init__(self, save_file_path="annotator_positions.pkl"):
        rospy.loginfo("Given save file path: " + save_file_path)
        if os.path.isfile(save_file_path):
            rospy.loginfo("File already exists, loading saved positions.")
            with open(save_file_path, "rb") as save_file:
                try:
                    self._positions = pickle.load(save_file)
                except EOFError:
                    # this can be caused if the file is empty.
                    self._positions = {}
                rospy.loginfo("File loaded...")
        else:
            self._positions = {}
        self._save_file_path = save_file_path
        self.__print_positions__()
        
        self._client = actionlib.SimpleActionClient('move_base/', MoveBaseAction)
        rospy.loginfo("Waiting for server...")
        self._client.wait_for_server()
        rospy.loginfo("Got connection.")

    def __print_positions__(self):
        rospy.loginfo("Current positions:")
        for k in self._positions:
            rospy.loginfo("\t" + k)

    def __save_file__(self):
        with open(self._save_file_path, "wb") as save_file:
            pickle.dump(self._positions, save_file, protocol=pickle.HIGHEST_PROTOCOL)
            # flush() saves file immediately instead of buffering changes.
            save_file.flush()

    def save_position(self, name, pose=None):
        if pose is None:
            pose = rospy.wait_for_message("/amcl_pose", PoseWithCovarianceStamped).pose.pose
        self._positions[name] = pose
        self.__save_file__()
    
    def get_position_names(self):
        return self._positions.keys()

    def get_position_items(self):
        return self._positions.items()

    def delete_position(self, name):
        if name in self._positions:
            self._positions.pop(name)
            self.__save_file__()

    def goto_position(self, name):
        if name in self._positions:
            position = self._positions[name]

            goal = MoveBaseGoal()
            goal.target_pose.pose = position
            goal.target_pose.header.frame_id = "map"

            self._client.send_goal(goal)
            self._client.wait_for_result()
    
    def exists(self, name):
        return name in self._positions
