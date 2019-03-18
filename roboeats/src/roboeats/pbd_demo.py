#!/usr/bin/env python

import actionlib
import pickle
import rospy
import robot_api
import numpy as np
import os
import tf
import tf.transformations as tft
import tf2_ros

from food_item import FoodItem
from server import RoboEatsServer

from pbd import Command, Program
from geometry_msgs.msg import PoseStamped
from ar_track_alvar_msgs.msg import AlvarMarkers
from robot_controllers_msgs.msg import QueryControllerStatesAction, QueryControllerStatesGoal, ControllerState
from roboeats.srv import StartSequenceRequest
from roboeats.srv import CreateFoodItemRequest

def pos_rot_str(pos, rot):
    return "pos: (%.5f, %.5f, %.5f), rot: (%.5f, %.5f, %.5f, %.5f)" % (pos[0], pos[1], pos[2], rot[0], rot[1], rot[2], rot[3])

def ps_str(ps):
    pos = ps.pose.position
    rot = ps.pose.orientation
    return "PoseStamped(pos: (%.5f, %.5f, %.5f), rot: (%.5f, %.5f, %.5f, %.5f), frame_id: %s)" % (pos.x, pos.y, pos.z, rot.x, rot.y, rot.z, rot.w, ps.header.frame_id)

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
    print("save-open-gripper|sog: save open gripper cmd in sequence")
    print("save-close-gripper|scg: save close gripper cmd in sequence")
    print("alias <i> <new_alias>: sets an alias for a cmd at index i")
    print("delete|d: deletes the last saved command")
    print("replace-frame|rf <alias> <new_frame>: replaces frames in poses with the given alias")
    print("run-program|rp: runs the current program sequence.")
    print("savef <file_path>: saves the program to the given file path.")
    print("loadf <file_path>: loads the program from the given file path.")
    print("print-program|ls|list: print the current program sequence")
    print("relax: relax the arm")
    print("unrelax: unrelaxes the arm")
    print("tags: print the available tag frames")
    print("stop: stops the demo")
    print("torso <height>: moves the torso to the specified height")
    print("head <pan> <tilt>: moves the head to the specified pan and tilt values")
    print(" ")
    print("======ROBOEATS SERVER COMMANDS=====")
    print("init: initializes the robot")
    print("attachl: attaches lunchbox obstacle")
    print("detachl: detaches lunchbox obstacle")
    print("obstacles1: start obstacles1 (microwave with lid closed)")
    print("obstacles2: start obstacles2 (microwave with lid open)")
    print("clear-obstacles: clears all obstacles")
    print("segment1a: start segment 1a")
    print("segment1b: start segment 1b")
    print("segment2: start segment 2")
    print("segment3: start segment 3")
    print("segment4: start segment 4")
    print("all-segments: run all segments")
    print("set-food-id: sets food id")
    print("list-foods: prints out foods")
    print("addf <name> <description> <id>: adds the food item")
    print("help: Show this list of commands")

def print_intro():
    print("Welcome to the program by demonstration!")
    print_commands()

class ArTagReader(object):
    def __init__(self):
        self.markers = []

    def callback(self, msg):
        try:
            # Filter to AR tag marker with ids of 0 or 15
            # so literally only the microwave and the lunchbox we want
            # to pick up.
            # This is necessary because Fetch detects AR tags that don't
            # exist with ids between 0 and 15 too (such as 3, 7, 9, etc.)
            self.markers = []
            for m in msg.markers:
                # if m.id == 0 or m.id == 15:
                self.markers.append(m)
            # self.markers = msg.markers
        except Exception as e:
            print("GOT EXCEPTISDOAUFHUASJFSADf?!@?#!@?!@?#!@?#!@")
    
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

def relax_arm(controller_client):
    goal = QueryControllerStatesGoal()
    state = ControllerState()
    state.name = 'arm_controller/follow_joint_trajectory'
    state.state = ControllerState.STOPPED
    goal.updates.append(state)
    controller_client.send_goal(goal)
    controller_client.wait_for_result()
    print("Arm is now relaxed.")

def main():
    rospy.init_node("annotator_node")
    print("starting node")
    wait_for_time()
    print("finished node")
    
    print("starting node 2")
    listener = tf.TransformListener()
    rospy.sleep(1)
    print("finished node 2")

    reader = ArTagReader()
    sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, callback=reader.callback)
    print('finished subscribing to ARTag reader')

    controller_client = actionlib.SimpleActionClient('query_controller_states', QueryControllerStatesAction)
    print('passed action lib')
    arm = robot_api.Arm()
    print('got arm')
    gripper = robot_api.Gripper()
    print('got gripper')
    torso = robot_api.Torso()
    print('got torso')
    head = robot_api.Head()
    print('got head')

    server = RoboEatsServer()

    print_intro()
    program = Program(arm, gripper, head, torso)
    print("Program created.")
    running = True
    food_id = None
    while running:
        user_input = raw_input(">>>")
        if not user_input:
            # string is empty, ignore
            continue
        args = user_input.split(" ")
        cmd = args[0]
        num_args = len(args) - 1
        if cmd == "create":
            program.reset()
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
                        # while True:
                        transform = listener.lookupTransform(frame, "base_link", rospy.Time(0))
                        rot = transform[1]
                        x, y, z, w = rot
                        print("stage 1: " + pos_rot_str(transform[0], transform[1]))
                        tag_T_base = create_transform_matrix(transform)
                        user_input = raw_input("saved base relative to the frame, move the arm and press enter when done")
                        transform = listener.lookupTransform("base_link", "wrist_roll_link", rospy.Time(0))
                        print("stage 2: " + pos_rot_str(transform[0], transform[1]))
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
                    print("Saved pose: " + ps_str(ps))
                    program.add_pose_command(ps, alias)
                    print("done")
                except Exception as e:
                    print("Exception:", e)
            else:
                print("No frame given.")
        elif cmd == "save-open-gripper" or cmd == "sog":
            program.add_open_gripper_command()
            gripper.open()
        elif cmd == "save-close-gripper" or cmd == "scg":
            program.add_close_gripper_command()
            gripper.close()
        elif cmd == "alias":
            if num_args == 2:
                i = int(args[1])
                alias = args[2]
                program.set_alias(i, alias)
            else:
                print("alias requires <i> <alias>")
        elif cmd == "delete" or cmd == "d":
            program.delete_last_command()
        elif cmd == "replace-frame" or cmd == "rf":
            if num_args == 2:
                alias = args[1]
                new_frame = args[2]
                program.replace_frame(alias, new_frame)
            else:
                print("Expected 2 arguments, got " + str(num_args))
        elif cmd == "run-program" or cmd == "rp":
            program.run(None)
            relax_arm(controller_client)
        elif cmd == "savef":
            if len(args) == 2:
                try:
                    fp = args[1]
                    if program is None:
                        print("There is no active program currently.")
                    else:
                        program.save_program(fp)
                except Exception as e:
                    print("Failed to save!\n", e)
            else:
                print("No save path given.")
        elif cmd == "loadf":
            if len(args) == 2:
                fp = args[1]
                if os.path.isfile(fp):
                    print("File " + fp + " exists. Loading...")
                    with open(fp, "rb") as load_file:
                        program.commands = pickle.load(load_file)
                    print("Program loaded...")
                    program.print_program()
            else:
                print("No frame given.")
        elif cmd == "print-program" or cmd == "ls" or cmd == "list":
            program.print_program()
        elif cmd == "relax":
            relax_arm(controller_client)
        elif cmd == "unrelax":
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
        elif cmd == "torso":
            if num_args == 1:
                height = float(args[1])
                program.add_set_height_command(height)
                torso.set_height(height)
            else:
                print("missing <height>")
        elif cmd == "head":
            if num_args == 2:
                pan = float(args[1])
                tilt = float(args[2])
                program.add_set_pan_tilt_command(pan, tilt)
                head.pan_tilt(pan, tilt)
            else:
                print("missing <pan> <tilt>")
        elif cmd == "init":
            server.init_robot()
        elif cmd == "attachl":
            server.attach_lunchbox()
        elif cmd == "detachl":
            server.remove_lunchbox()
        elif cmd == "obstacles1":
            server.start_obstacles_1()
        elif cmd == "obstacles2":
            server.start_obstacles_2()
        elif cmd == "clear-obstacles":
            server.clear_obstacles()
        elif cmd == "segment1a":
            server.start_segment1a(food_id)
        elif cmd == "segment1b":
            server.start_segment1b(food_id)
        elif cmd == "segment2":
            server.start_segment2(food_id)
        elif cmd == "segment3":
            server.start_segment3(food_id)
        elif cmd == "segment4":
            server.start_segment4(food_id)
        elif cmd == "all-segments":
            rqst = StartSequenceRequest()
            rqst.id = food_id
            server.handle_start_sequence(rqst)
        elif cmd == "set-food-id":
            if num_args == 1:
                food_id = int(args[1])
            else:
                print("Requires <food_id>")
        elif cmd == "addf":
            if num_args == 3:
                rqst = CreateFoodItemRequest()
                rqst.name = args[1]
                rqst.description = args[2]
                rqst.id = int(args[3])
                server.handle_create_food_item(rqst)
            else:
                print("Requires <name> <description> <id>")
        elif cmd == "list-foods":
            server.__print_food_items__()
        else:
            print("NO SUCH COMMAND: " + cmd)
        print("")

if __name__ == '__main__':
  main()
