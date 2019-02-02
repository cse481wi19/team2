#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
import actionlib
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseGoal, MoveBaseAction
import pickle
import os.path

class Position(object):
    def __init__(self):
        # self.__position = None
        print "constructor"
        # self._subscriber = rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, callback=self.callback)
        self.__client = actionlib.SimpleActionClient('move_base/', MoveBaseAction)
        print "now wait for server"
        self.__client.wait_for_server()
        print "done with constructor"

    # def callback(self, msg):
    #     self.__position = msg.pose.pose

    def most_recent_position(self):
        return rospy.wait_for_message("/amcl_pose", PoseWithCovarianceStamped)

    def move_robot(self, point):
        goal = MoveBaseActionGoal()
        
        goal_trajectory = MoveBaseGoal()
        pose_stamped = PoseStamped()

        pose_stamped.pose = point.pose.pose
        pose_stamped.header.frame_id = "map"

        goal_trajectory.target_pose = pose_stamped
        goal.goal = goal_trajectory

        goal.header.frame_id = "map"
        self.__client.send_goal(goal_trajectory)
        self.__client.wait_for_result()



def main():
    rospy.init_node ('annotator')
    position = Position()
    saved_positions = dict()
    print ("Welcome to the map annotator!")
    if os.path.isfile('positions.pkl'):
        print "Found position storage file"
        with open('positions.pkl', 'rb') as saved_output_positions:
            saved_positions = pickle.load(saved_output_positions)
    while True:
        print("Commands: ")
        print("list: List saved poses") 
        print ("save <name>: Save the robot's current pose as <name> Overwrites if <name> already exits")
        print ("delete <name>")
        print("gotto <name>")
        print("help: Show this lsit of commands")
        command = str(raw_input())
        split_command = command.split(" ")
        if split_command[0] == 'save':
            command = command.replace("save ", "", 1).strip()
            pos_to_save = position.most_recent_position()
            print "Saving complete"
            if pos_to_save is None:
                print "position not initialized"
            else:
                print "calling save"
                saved_positions[command] = pos_to_save
                print "saving complete"
        elif split_command[0] == 'goto':
            command = command.replace("goto ", "", 1).strip()
            point_to_move = saved_positions[command]
            position.move_robot(point_to_move)
        elif split_command[0] == 'stop':
            break
    print "we are done!"
    with open('positions.pkl', 'wb') as output:
        pickle.dump(saved_positions, output, pickle.HIGHEST_PROTOCOL)

    # rospy.spin()

if __name__ == '__main__':
  main()