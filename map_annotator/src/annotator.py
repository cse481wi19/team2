#!/usr/bin/env python                                                                                  
                                                                                                       
import rospy     
import pickle
import actionlib
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseGoal, MoveBaseAction

class Annotator(object):                                
    DEFAULT_SAVE_FILE_NAME = "annotator_positions.pkl"

    def __init__(self, save_file_path=None):
        if save_file is None:
            print("Creating save file: ", DEFAULT_SAVE_FILE_NAME)
            self._save_file = open(DEFAULT_SAVE_FILE_NAME, "wb")
            self._positions = {}
        else:
            print("Given save file: ", save_file_path)
            self._save_file = open(save_file_path, "wb")
            self._positions = pickle.load(self._save_file)
        self._client = actionlib.SimpleActionClient('move_base/', MoveBaseAction)
        self._client.wait_for_server()

    def __save_file__(self):
        pickle.dump(self._positions, self._save_file)

    def save_position(self, name):
        pose = rospy.wait_for_message("/amcl_pose", PoseWithCovarianceStamped)
        self._positions[name] = pose
        self.__save_file__()
    
    def get_positions(self):
        return self._positions.keys()

    def delete_position(self, name):
        if name in self._positions:
            self._positions.pop(name)
            self.__save_file__()

    def goto_position(self, name):
        if name in self._positions:
            position = self._positions[name]

            goal = MoveBaseActionGoal()
        
            goal_trajectory = MoveBaseGoal()
            pose_stamped = PoseStamped()

            pose_stamped.pose = position.pose.pose
            pose_stamped.header.frame_id = "map"

            goal_trajectory.target_pose = pose_stamped
            goal.goal = goal_trajectory

            goal.header.frame_id = "map"
            self._client.send_goal(goal_trajectory)
            self._client.wait_for_result()
