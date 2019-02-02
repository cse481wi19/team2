#!/usr/bin/env python

import rospy

from map_annotator import Annotator

def wait_for_time():                                              
    """Wait for simulated time to begin.                          
    """                                                           
    while rospy.Time().now().to_sec() == 0:                       
        pass

def print_commands():
    print("Commands:")
    print("list: List saved poses.")
    print("save <name>: Save the robot's current pose as <name>. Overwrites if <name> already exists.")
    print("delete <name>: Delete the pose given by <name>.")
    print("goto <name>: Sends the robot to the pose given by <name>.")
    print("help: Show this list of commands")

def print_intro():
    print("Welcome to the map annotator!")
    print_commands()

def main():
    rospy.init_node("annotator_node")
    wait_for_time()
    annotator = Annotator()
    running = True
    while running:
        user_input = raw_input(">>>")
        if not user_input:
            # string is empty, ignore
            continue
        args = user_input.split(" ", 1)
        cmd = args[0]
        if cmd == "list":
            positions = annotator.get_positions()
            print("Poses:")
            for position in positions:
                print("\t" + position)
        elif cmd == "save":
            if len(args) == 2:
                name = args[1]
                annotator.save_position(name)
                print("Position '" + name + "' was saved!")
            else:
                print("No name given.")
        elif cmd == "delete":
            if len(args) == 2:
                name = args[1]
                annotator.delete_position(name)
                print("Position '" + name + "' was deleted!")
            else:
                print("No name given.")
        elif cmd == "goto":
            if len(args) == 2:
                name = args[1]
                annotator.goto_position(name)
                print("Going to position '" + name + "'.")
            else:
                print("No name given.")
        elif cmd == "help":
            print_commands()
        elif cmd == "stop":
            running = False
        print("")

if __name__ == '__main__':
  main()
