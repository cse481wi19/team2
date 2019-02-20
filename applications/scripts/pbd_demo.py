#!/usr/bin/env python

import pickle
import rospy
import robot_api
import numpy as np
import os
import tf
import tf.transformations as tft

from geometry_msgs.msg import PoseStamped
from ar_track_alvar_msgs.msg import AlvarMarkers

def wait_for_time():                                              
    """Wait for simulated time to begin.                          
    """                                                           
    while rospy.Time().now().to_sec() == 0:                       
        pass

def print_commands():
    print("Commands:")
    print("create: starts the creation of a program")
    print("save-pose <frame>: saves the current pose relative to the given frame. <frame> can either be relative to the base or the tag.")
    print("save-open-gripper: save open gripper cmd in sequence")
    print("save-close-gripper: save close gripper cmd in sequence")
    print("run-program: runs the current program sequence.")
    print("save-program <file_path>: saves the program to the given file path.")
    print("load-program <file_path>: loads the program from the given file path.")
    print("print-program: print the current program sequence")
    print("tags: print the available tag frames")
    print("stop: stops the demo")
    print("help: Show this list of commands")

def print_intro():
    print("Welcome to the program by demonstration!")
    print_commands()

class Command(object):
    POSE = 1
    OPEN_GRIPPER = 2
    CLOSE_GRIPPER = 3

    NUM_TO_CMD = {1:"Pose", 2:"Open gripper", 3:"Close gripper"}
    def __init__(self, type, pose_stamped=None):
        self.type = type
        self.pose_stamped = pose_stamped

class Program(object):
    def __init__(self):
        self.commands = []

    def add_pose_command(self, ps):
        self.commands.append(Command(Command.POSE, pose_stamped=ps))
        

    def add_open_gripper_command(self):
        self.commands.append(Command(Command.OPEN_GRIPPER))
    
    def add_close_gripper_command(self):
        self.commands.append(Command(Command.CLOSE_GRIPPER))

    def save_program(self, fp):
        with open(fp, "wb") as save_file:
            pickle.dump(self, save_file)

    def print_program(self):
        for i, command in enumerate(self.commands):
            print(i, Command.NUM_TO_CMD[command.type])

    def run(self):
        arm = robot_api.Arm()
        gripper = robot_api.Gripper()
        listener = tf.TransformListener()
        rospy.sleep(1)
        for command in self.commands:
            if command.type == Command.POSE:
                print(command.pose_stamped)
                ps = command.pose_stamped
                curr_frame = command.pose_stamped.header.frame_id
                if curr_frame != "base_link":
                    (pos, rot) = listener.lookupTransform("base_link", curr_frame, rospy.Time(0))
                    base_T_frame_matrix = tft.quaternion_matrix(rot)
                    base_T_frame_matrix[0, 3] = pos[0]
                    base_T_frame_matrix[1, 3] = pos[1]
                    base_T_frame_matrix[2, 3] = pos[2]

                    ori = ps.pose.orientation
                    frame_T_gripper_matrix = tft.quaternion_matrix([ori.x, ori.y, ori.z, ori.w])
                    frame_T_gripper_matrix[0, 3] = ps.pose.position.x
                    frame_T_gripper_matrix[1, 3] = ps.pose.position.y
                    frame_T_gripper_matrix[2, 3] = ps.pose.position.z

                    ans = np.dot(base_T_frame_matrix, frame_T_gripper_matrix)
                    ans2 = tft.quaternion_from_matrix(ans)
                    ps = PoseStamped()
                    ps.pose.position.x = ans[0, 3]
                    ps.pose.position.y = ans[1, 3]
                    ps.pose.position.z = ans[2, 3]
                    ps.pose.orientation.x = ans2[0]
                    ps.pose.orientation.y = ans2[1]
                    ps.pose.orientation.z = ans2[2]
                    ps.pose.orientation.w = ans2[3]
                    ps.header.frame_id = "base_link"
                arm.move_to_pose(ps)
            elif command.type == Command.OPEN_GRIPPER:
                gripper.open()
            elif command.type == Command.CLOSE_GRIPPER:
                gripper.close()
            else:
                print("UNKNOWN COMMAND " + str(command.type))
    

class ArTagReader(object):
    def __init__(self):
        self.markers = []

    def callback(self, msg):
        self.markers = msg.markers
    
    def get_available_tag_frames(self):
        tag_frames = []
        for marker in self.markers:
            tag_frames.append("ar_marker_" + str(marker.id))
        return tag_frames



def main():
    rospy.init_node("annotator_node")
    wait_for_time()
    
    listener = tf.TransformListener()
    rospy.sleep(1)

    reader = ArTagReader()
    sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, callback=reader.callback)

    print_intro()
    program = None
    running = True
    while running:
        user_input = raw_input(">>>")
        if not user_input:
            # string is empty, ignore
            continue
        args = user_input.split(" ", 1)
        cmd = args[0]
        if cmd == "create":
            program = Program()
        elif cmd == "save-pose":
            if len(args) == 2:
                frame = args[1]
                (pos, rot) = listener.lookupTransform(frame, "gripper_link", rospy.Time(0))
                print(str(pos), str(rot))

                # Maybe we need some kind of offset?
                wrist_link_xoff = 0.166

                ps = PoseStamped()
                ps.header.frame_id = frame
                ps.pose.position.x = pos[0]
                ps.pose.position.y = pos[1]
                ps.pose.position.z = pos[2]

                ps.pose.orientation.x = rot[0]
                ps.pose.orientation.y = rot[1]
                ps.pose.orientation.z = rot[2]
                ps.pose.orientation.w = rot[3]
                program.add_pose_command(ps)
            else:
                print("No frame given.")
        elif cmd == "save-open-gripper":
            program.add_open_gripper_command()
        elif cmd == "save-close-gripper":
            program.add_close_gripper_command()
        elif cmd == "run-program":
            program.run()
        elif cmd == "save-program":
            if len(args) == 2:
                fp = args[1]
                if program is None:
                    print("There is no active program currently.")
                else:
                    program.save_program(fp)
            else:
                print("No save path given.")
        elif cmd == "load-program":
            if len(args) == 2:
                fp = args[1]
                if os.path.isfile(fp):
                    print("File " + fp + " exists. Loading...")
                    with open(fp, "rb") as load_file:
                        program = pickle.load(load_file)
                    print("Program loaded...")
                    program.print_program()
            else:
                print("No frame given.")
        elif cmd == "print-program":
            program.print_program()
        elif cmd == "tags":
            for frame in reader.get_available_tag_frames():
                print("\t" + frame)
        elif cmd == "help":
            print_commands()
        elif cmd == "stop":
            running = False
        print("")

if __name__ == '__main__':
  main()
