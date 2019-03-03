#!/usr/bin/env python

import actionlib
import pickle
import rospy
import robot_api
import numpy as np
import os
import tf
import tf.transformations as tft

from geometry_msgs.msg import PoseStamped
from ar_track_alvar_msgs.msg import AlvarMarkers
from robot_controllers_msgs.msg import QueryControllerStatesAction, QueryControllerStatesGoal, ControllerState

def wait_for_time():                                              
    """Wait for simulated time to begin.                          
    """                                                           
    while rospy.Time().now().to_sec() == 0:                       
        pass

def print_commands():
    print("Commands:")
    print("create: starts the creation of a program")
    print("save-pose|sp <frame>: saves the current pose relative to the given frame. <frame> can either be relative to the base or the tag.")
    print("save-pose|sp <alias> <frame>: saves the current pose relative to the given frame with an alias. <frame> can either be relative to the base or the tag.")
    print("save-open-gripper: save open gripper cmd in sequence")
    print("save-close-gripper: save close gripper cmd in sequence")
    print("replace-frame|rf <alias> <new_frame>: replaces frames in poses with the given alias")
    print("run-program: runs the current program sequence.")
    print("save-program <file_path>: saves the program to the given file path.")
    print("load-program <file_path>: loads the program from the given file path.")
    print("print-program|ls|list: print the current program sequence")
    print("arm-relax: relax the arm")
    print("arm-unrelax: unrelaxes the arm")
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

    NUM_TO_CMD = {POSE:"Pose", OPEN_GRIPPER:"Open gripper", CLOSE_GRIPPER:"Close gripper"}
    def __init__(self, type, pose_stamped=None, alias=None):
        self.type = type
        self.pose_stamped = pose_stamped
        self.alias = alias

    def __str__(self):
        if self.type == self.POSE:
            return "Command(type=%s, alias=%s, frame_id=%s)" % (self.NUM_TO_CMD[self.type], str(self.alias), str(self.pose_stamped.header.frame_id))
        else:
            return "Command(type=%s)" % (self.NUM_TO_CMD[self.type])
        

class Program(object):
    def __init__(self):
        self.commands = []

    def add_pose_command(self, ps, alias):
        self.commands.append(Command(Command.POSE, pose_stamped=ps, alias=alias))
        
    def add_open_gripper_command(self):
        self.commands.append(Command(Command.OPEN_GRIPPER))
    
    def add_close_gripper_command(self):
        self.commands.append(Command(Command.CLOSE_GRIPPER))

    def save_program(self, fp):
        with open(fp, "wb") as save_file:
            pickle.dump(self, save_file)

    def print_program(self):
        for i, command in enumerate(self.commands):
            print(i, str(command))

    def replace_frame(self, alias, new_frame):
        for i, command in enumerate(self.commands):
            if command.alias == alias:
                old_frame = command.pose_stamped.header.frame_id
                command.pose_stamped.header.frame_id = new_frame
                print("Command %d: Replaced '%s' with '%s'" % (i, old_frame, new_frame))
        print("New program:")
        self.print_program()

    def run(self):
        try:
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
                        listener.waitForTransform("base_link", curr_frame, rospy.Time(), rospy.Duration(4.0))
                        while not rospy.is_shutdown():
                            try:
                                now = rospy.Time.now()
                                listener.waitForTransform("base_link", curr_frame, now, rospy.Duration(4.0))
                                (pos, rot) = listener.lookupTransform("base_link", curr_frame, now)
                                break
                            except:
                                pass
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
                rospy.sleep(1)
                print("Run succeeded!")
        except Exception as e:
            print("Run failed!")
            print(e)
    

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

def create_transform_matrix(transform):
    # transform is tuple (pos, rot)
    (pos, rot) = transform
    matrix = tft.quaternion_matrix(rot)
    matrix[0, 3] = pos[0]
    matrix[1, 3] = pos[1]
    matrix[2, 3] = pos[2]
    return matrix

def main():
    rospy.init_node("annotator_node")
    wait_for_time()
    
    listener = tf.TransformListener()
    rospy.sleep(1)

    reader = ArTagReader()
    sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, callback=reader.callback)

    controller_client = actionlib.SimpleActionClient('query_controller_states', QueryControllerStatesAction)

    print_intro()
    program = Program()
    print("Program created.")
    running = True
    while running:
        user_input = raw_input(">>>")
        if not user_input:
            # string is empty, ignore
            continue
        args = user_input.split(" ", 2)
        cmd = args[0]
        num_args = len(args) - 1
        if cmd == "create":
            program = Program()
            print("Program created.")
        elif cmd == "save-pose" or cmd == "sp":
            if num_args >= 1:
                try:
                    if num_args == 1:
                        alias = None
                        frame = args[1]
                    elif num_args == 2:
                        alias = args[1]
                        frame = args[2]
                    if frame == "base_link":
                        (pos, rot) = listener.lookupTransform(frame, "wrist_roll_link", rospy.Time(0))
                        print(str(pos), str(rot))

                        ps = PoseStamped()
                        ps.header.frame_id = frame
                        ps.pose.position.x = pos[0]
                        ps.pose.position.y = pos[1]
                        ps.pose.position.z = pos[2]

                        ps.pose.orientation.x = rot[0]
                        ps.pose.orientation.y = rot[1]
                        ps.pose.orientation.z = rot[2]
                        ps.pose.orientation.w = rot[3]
                    else:
                        transform = listener.lookupTransform(frame, "base_link", rospy.Time(0))
                        tag_T_base = create_transform_matrix(transform)

                        user_input = raw_input("saved base relative to the frame, move the arm and press enter when done")
                        transform = listener.lookupTransform("base_link", "wrist_roll_link", rospy.Time(0))
                        base_T_gripper = create_transform_matrix(transform)
                        
                        ans = np.dot(tag_T_base, base_T_gripper)
                        ans2 = tft.quaternion_from_matrix(ans)
                        
                        ps = PoseStamped()
                        ps.pose.position.x = ans[0, 3]
                        ps.pose.position.y = ans[1, 3]
                        ps.pose.position.z = ans[2, 3]
                        ps.pose.orientation.x = ans2[0]
                        ps.pose.orientation.y = ans2[1]
                        ps.pose.orientation.z = ans2[2]
                        ps.pose.orientation.w = ans2[3]
                        ps.header.frame_id = frame
                    program.add_pose_command(ps, alias)
                    print("done")
                except tf.LookupException:
                    print("Failed to lookup given frame '%s'" % (frame))
            else:
                print("No frame given.")
        elif cmd == "save-open-gripper":
            program.add_open_gripper_command()
        elif cmd == "save-close-gripper":
            program.add_close_gripper_command()
        elif cmd == "replace-frame" or cmd == "rf":
            if num_args == 2:
                alias = args[1]
                new_frame = args[2]
                program.replace_frame(alias, new_frame)
            else:
                print("Expected 2 arguments, got " + str(num_args))
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
        elif cmd == "print-program" or cmd == "ls" or cmd == "list":
            program.print_program()
        elif cmd == "arm-relax":
            goal = QueryControllerStatesGoal()
            state = ControllerState()
            state.name = 'arm_controller/follow_joint_trajectory'
            state.state = ControllerState.STOPPED
            goal.updates.append(state)
            controller_client.send_goal(goal)
            controller_client.wait_for_result()
            print("Arm is now relaxed.")
        elif cmd == "arm-unrelax":
            goal = QueryControllerStatesGoal()
            state = ControllerState()
            state.name = 'arm_controller/follow_joint_trajectory'
            state.state = ControllerState.RUNNING
            goal.updates.append(state)
            controller_client.send_goal(goal)
            controller_client.wait_for_result()
            print("Arm is now unrelaxed.")
        elif cmd == "tags":
            for frame in reader.get_available_tag_frames():
                print("\t" + frame)
        elif cmd == "help":
            print_commands()
        elif cmd == "stop":
            running = False
        else:
            print("NO SUCH COMMAND: " + cmd)
        print("")

if __name__ == '__main__':
  main()
