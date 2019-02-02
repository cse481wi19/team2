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
    annotator = Annotator()
    running = True
    while running:
        user_input = raw_input(">>>")
        if not user_input:
            # string is empty, ignore
            continue
        args = user_input.split(" ", 1)
        print arg, user_input
        cmd = args[0]
        if cmd == "list":
            positions = annotator.get_positions()
            print("Poses:")
            for position in positions:
                print("\t", position)
        elif cmd == "save":
            name = args[1]
            annotator.save_position(name)
            print("Position '", name, "'was saved!")
        elif cmd == "delete":
            name = args[1]
            annotator.delete_position(name)
            print("Position '", name, "'was deleted!")
        elif cmd == "goto":
            name = args[1]
            pass
        elif cmd == "help":
            print_commands()
    # rospy.init_node('annotator_node')
    # wait_for_time()
    # marker_publisher = rospy.Publisher('visualization_marker', Marker, queue_size=10)
    # rospy.sleep(0.5)
    # show_text_in_rviz(marker_publisher, "RoboEats is cool.")

if __name__ == '__main__':
  main()