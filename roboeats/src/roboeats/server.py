#!/usr/bin/env python

import rospy
import os
import pickle

import robot_api
import tf
import tf.transformations as tft

from geometry_msgs.msg import Pose, PoseWithCovarianceStamped

from moveit_python import PlanningSceneInterface
from map_annotator import Annotator
from pbd import Program, Command

from food_item import FoodItem

from roboeats.msg import FoodItems
from roboeats.srv import CreateFoodItem, CreateFoodItemResponse
from roboeats.srv import RemoveFoodItem, RemoveFoodItemResponse
from roboeats.srv import StartSequence, StartSequenceResponse


def wait_for_time():
    """Wait for simulated time to begin.                          
    """
    while rospy.Time().now().to_sec() == 0:
        pass

FOOD_ITEMS_TOPIC = "/roboeats/food_items"

class RoboEatsServer(object):
    DEFAULT_TORSO_HEIGHT = 0.4

    MICROWAVE_LOCATION_NAME = "microwave_location"
    DROPOFF_LOCATION_NAME = "dropoff_location"
    FOOD_ALIAS = "food_id"

    def __init__(self, save_file_path="food_items.pkl", nav_file_path="annotator_positions.pkl"):
        self._food_items_pub = rospy.Publisher(FOOD_ITEMS_TOPIC,
                                               FoodItems, queue_size=10, latch=True)
        rospy.loginfo("Given save file path: " + save_file_path)
        if os.path.isfile(save_file_path):
            rospy.loginfo("File already exists, loading saved positions.")
            with open(save_file_path, "rb") as save_file:
                try:
                    self._food_items = pickle.load(save_file)
                except EOFError:
                    # this can be caused if the file is empty.
                    self._food_items = {}
                rospy.loginfo("File loaded...")
        else:
            rospy.loginfo("File doesn't exist.")
            self._food_items = {}
        self.__print_food_items__()
        self._save_file_path = save_file_path
        self.__pub_food_items__()

        rospy.loginfo("initializing arm...")
        self.arm = robot_api.Arm()

        rospy.loginfo("initializing gripper...")
        self.gripper = robot_api.Gripper()


        rospy.loginfo("initializing head...")
        self.head = robot_api.Head()

        rospy.loginfo("initializing torso...")
        self.torso = robot_api.Torso()

        rospy.loginfo("initializing planning scene...")
        self.planning_scene = PlanningSceneInterface('base_link')

        rospy.loginfo("Starting map annotator...")
        # We should expect the nav file given to contain the annotated positions:
        #   MICROWAVE_LOCATION_NAME - starting location in front of the microwave.
        #   DROPOFF_LOCATION_NAME - ending dropoff location.
        
        # TODO: Remember to uncomment this section when we get the map working.
        # self._map_annotator = Annotator(save_file_path=nav_file_path)
        # if not self._map_annotator.exists(self.MICROWAVE_LOCATION_NAME):
        #     rospy.logwarn("Annotator is missing location '%s'" % 
        #                   (self.MICROWAVE_LOCATION_NAME))
        # if not self._map_annotator.exists(self.DROPOFF_LOCATION_NAME):
        #     rospy.logwarn("Annotator is missing location '%s'" %
        #                   (self.DROPOFF_LOCATION_NAME))
        rospy.loginfo("Initialization finished...")

    def __print_food_items__(self):
        rospy.loginfo("Current food items:")
        for f in self._food_items.values():
            rospy.loginfo("\t" + str(f))

    def __save_file__(self):
        """
        Saves the pickle file containing food item information.
        """

        with open(self._save_file_path, "wb") as save_file:
            pickle.dump(self._food_items, save_file,
                        protocol=pickle.HIGHEST_PROTOCOL)
            # flush() saves file immediately instead of buffering changes.
            save_file.flush()

    def __pub_food_items__(self):
        """
        Publishes the current list of food items to the FOOD_ITEMS_TOPIC topic.
        """
        food_items = FoodItems()
        if len(self._food_items) > 0:
            food_items.names, food_items.descriptions, food_items.ids = zip(*[(f.name, f.description, f.id) for f in self._food_items.values()])
        self._food_items_pub.publish(food_items)

    def __food_list_modified__(self):
        """
        Helper method which does any actions that're needed after the food item dict has been modified.
        """
        self.__save_file__()
        self.__pub_food_items__()

    def __remove_food_item__(self, id):
        if id in self._food_items:
            self._food_items.pop(id)
            self.__food_list_modified__()

    def handle_create_food_item(self, request):
        """
        input: request(name, description, id)

        Creates and adds the given food item.
        If a food item with the same ID already exists, then it will be overridden by this food item.
        """
        rospy.loginfo(request)
        self._food_items[request.id] = FoodItem(request.name, request.description, request.id)
        self.__food_list_modified__()
        return CreateFoodItemResponse()

    def handle_remove_food_item(self, request):
        """
        input: request(id)

        Removes the food item with the given id iff it exists.
        """
        self.__remove_food_item__(request.id)
        return RemoveFoodItemResponse()

    def __food_id_to_ar_frame__(self, id):
        return "ar_marker_" + str(id)

    def __load_program_and_run__(self, program_fp, id):
        """Load a pickle file from the given file path and run it with respect to the given food id.
        
        Arguments:
            program_fp {str} -- program file path
            id {int} -- food id
        """
        if os.path.isfile(program_fp):
            rospy.loginfo("File " + program_fp + " exists. Loading...")
            with open(program_fp, "rb") as load_file:
                program = Program(self.arm, self.gripper, self.head, self.torso)
                program.commands = pickle.load(load_file)
                rospy.loginfo("Program loaded...")
                ar_marker_frame = self.__food_id_to_ar_frame__(id)
                rospy.loginfo("Program before changes:")
                program.print_program()

                # Make the program relative to this food item
                rospy.loginfo("Program after changes:")
                program.replace_frame(self.FOOD_ALIAS, ar_marker_frame)
                program.run()

        else:
            rospy.logerr("Program from given file path does not exist: " + program_fp)
            raise Exception

    def init_robot(self):
        """
            0a. Move torso to default position
            0b. reset head
            0c. open gripper
            0d. move arm to starting pos (start_pos.pkl)
        """
        rospy.loginfo("STARTING SEGMENT 1")
        rospy.loginfo("0a. Move torso to default position")
        torso = robot_api.Torso()
        torso.set_height(0.4)
        rospy.sleep(2)

        rospy.loginfo("0b. reset head")
        head = robot_api.Head()
        head.pan_tilt(-0.1, 0.57)
        rospy.sleep(2)

        rospy.loginfo("0c. open gripper")
        self.gripper.open()

        rospy.loginfo("0d. Move arm to starting pos")
        self.__load_program_and_run__("start_pos.pkl", id)
        rospy.sleep(1.5)

    def clear_obstacles(self):
        self.planning_scene.clear()
        self.planning_scene.removeCollisionObject('table')
        self.planning_scene.removeCollisionObject('floor')
        self.planning_scene.removeCollisionObject('lunchbox')
        self.planning_scene.removeAttachedObject('lunchbox')

    def start_obstacles_1(self):
        """
        This scene has the microwave in a closed position.
        """
        self.planning_scene.clear()
        self.planning_scene.removeCollisionObject('table')
        self.planning_scene.removeCollisionObject('floor')
        self.planning_scene.addBox('floor', 2, 2, 0.01, 0, 0, 0.01/2)
        table_height = 0.767
        table_width = 0.7
        table_x = 0.95
        self.planning_scene.addBox('table', table_width, 2, table_height, table_x, 0, table_height/2)
        self.planning_scene.addBox('robot_base', 0.54, 0.52, 0.37, 0, 0, 0.37/2)

        microwave_height = 0.28
        microwave_width = 0.48
        # microwave_depth = 0.33
        microwave_depth = 0.27
        microwave_x = 0.97
        microwave_z = 0.06
        microwave_y = 0.18
        self.planning_scene.addBox('microwave', microwave_depth, microwave_width, microwave_height, microwave_x, microwave_y, table_height + microwave_z + microwave_height/2)
    
    def start_obstacles_2(self):
        """
        this scene has the microwave in an open position with the lid open.
        """
        self.planning_scene.clear()
        self.planning_scene.removeCollisionObject('table')
        self.planning_scene.removeCollisionObject('floor')
        self.planning_scene.addBox('floor', 2, 2, 0.01, 0, 0, 0.01/2)
        table_height = 0.767
        table_width = 0.7
        table_x = 0.95
        self.planning_scene.addBox('table', table_width, 2, table_height, table_x, 0, table_height/2)
        self.planning_scene.addBox('robot_base', 0.54, 0.52, 0.37, 0, 0, 0.37/2)

        microwave_height = 0.28
        microwave_width = 0.48
        microwave_depth = 0.27
        microwave_x = 0.97    
        microwave_y = 0.18
        microwave_z = 0.06
        

        microwave_side_height = 0.2
        microwave_r_width = 0.04 + 0.05
        microwave_r_y = microwave_y - 0.19
        microwave_l_width = 0.035
        microwave_l_y = microwave_y + 0.2
        microwave_bottom_height = 0.05
        microwave_top_height = 0.04
        microwave_back_depth = 0.03
        microwave_back_x = table_x + (microwave_depth / 2) + (microwave_back_depth/2)
        microwave_door_width = 0.04
        microwave_door_x = microwave_x - 0.33
        microwave_door_y = microwave_l_y + 0.05

        self.planning_scene.addBox('microwave_top', microwave_depth, microwave_width, microwave_top_height, microwave_x, microwave_y, table_height + microwave_z + microwave_bottom_height + microwave_side_height + (microwave_top_height/2))
        self.planning_scene.addBox('microwave_bottom', microwave_depth, microwave_width, microwave_bottom_height, microwave_x, microwave_y, table_height + microwave_z + (microwave_bottom_height/2) - 0.015)
        self.planning_scene.addBox('microwave_side_r', microwave_depth, microwave_r_width, microwave_side_height, microwave_x, microwave_r_y, table_height + microwave_z + microwave_bottom_height + (microwave_side_height/2))
        self.planning_scene.addBox('microwave_side_l', microwave_depth, microwave_l_width, microwave_side_height, microwave_x, microwave_l_y, table_height + microwave_z + microwave_bottom_height +  microwave_side_height/2)
        self.planning_scene.addBox('microwave_back', microwave_back_depth, microwave_width, microwave_height, microwave_back_x, microwave_y, table_height + microwave_z + microwave_height/2)
        self.planning_scene.addBox('microwave_door', 0.42, microwave_door_width, microwave_height + 0.01, microwave_door_x, microwave_door_y, table_height + microwave_z + microwave_height/2 + 0.005)

    def attach_lunchbox(self):
        self.remove_lunchbox()
        frame_attached_to = 'gripper_link'
        frames_okay_collide_with = ['gripper_link', 'l_gripper_finger_link', 'r_gripper_finger_link', 'wrist_roll_link']
        lunchbox_x_offset = 0.1
        self.planning_scene.attachBox("lunchbox", 0.10, 0.10, 0.02, lunchbox_x_offset, 0, 0, frame_attached_to, frames_okay_collide_with)

    def remove_lunchbox(self):
        self.planning_scene.removeAttachedObject('lunchbox')
        self.planning_scene.removeCollisionObject('lunchbox')

    def start_segment1a(self, id):
        """
        (Segment 1a)
            0. Initialize robot
            1. (OMITTED) Move to start pose
            2. Open microwave (p2.pkl) (done)
            2b. Move microwave lid (p2b.pkl) (done)
        """
        # if id in self._food_items:
        rospy.loginfo("0. Initialize robot")
        self.init_robot()

        self.start_obstacles_2()
        rospy.sleep(4)

        # rospy.loginfo("1. Move to start pose")
        # self._map_annotator.goto_position(self.MICROWAVE_LOCATION_NAME)
        # rospy.sleep(2)

        rospy.loginfo("2. Open microwave")
        self.__load_program_and_run__("p2.pkl", id)
        rospy.sleep(1.5)

        rospy.loginfo("2b. Move microwave lid")
        self.__load_program_and_run__("p2b.pkl", id)
        rospy.sleep(1.5)

        rospy.loginfo("FINISHED SEGMENT 1a")
        # else:
        #     print("Food item " + str(id) + " does not exist.")

    def start_segment1b(self, id):
        """
        (Segment 1b)
            3. Grab lunchbox (p1.pkl) (done - but redo if have enough time)
            4. Put it into microwave (p3.pkl) (done-iffy)
            5. Close microwave (p4a.pkl, p4b.pkl) (done) <- needs to be split so we can change planning scenes

        """
        # if id in self._food_items:
        self.start_obstacles_2()
        rospy.sleep(4)

        rospy.loginfo("STARTING SEGMENT 1b")
        rospy.loginfo("3. Grab lunchbox")
        self.__load_program_and_run__("p1.pkl", id)
        rospy.sleep(1.5)

        self.attach_lunchbox()
        rospy.sleep(2)

        rospy.loginfo("4. Put it into microwave")
        self.__load_program_and_run__("p3.pkl", id)
        rospy.sleep(1.5)

        self.remove_lunchbox()
        # else:
        #     print("Food item " + str(id) + " does not exist.")

    def start_segment1c(self, id):
        self.start_obstacles_2()
        rospy.sleep(4)

        rospy.loginfo("5a. Close microwave pt. 1")
        self.__load_program_and_run__("p4a.pkl", id)
        rospy.sleep(1.5)

        rospy.loginfo("5b. Changing obstacles...")
        self.start_obstacles_1()
        rospy.sleep(4)

        rospy.loginfo("5b. Close microwave pt. 2")
        self.__load_program_and_run__("p4b.pkl", id)
        rospy.sleep(1.5)
        rospy.loginfo("FINISHED SEGMENT 1c")

    def start_segment2(self, id):
        """
        (Segment 2)
            6. Enter time (1 min) & 7. Start microwave (p5.pkl)
            8. Wait for food to finish microwaving
            9. Wait for cooldown
        """
        # if id in self._food_items:
        rospy.loginfo("STARTING SEGMENT 2")
        rospy.loginfo("6. Enter time(1 min)")
        self.__load_program_and_run__("p5.pkl", id)

        rospy.loginfo("8. Wait for food to finish microwaving (in seconds)")
        rospy.sleep(5)

        rospy.loginfo("9. Wait for cooldown (in seconds)")
        # rospy.sleep(40) 
        rospy.loginfo("FINISHED SEGMENT 2")
        # else:
        #     print("Food item " + str(id) + " does not exist.")

    def start_segment3a(self, id):
        """
        (Segment 3)
            10a. Open microwave (p2.pkl) (done)
            10b. Move microwave lid (p2b.pkl) (done)
        """
        # if id in self._food_items:
        self.start_obstacles_2()
        rospy.sleep(4)
        
	rospy.loginfo("PRE INITIALIZING FOR SEGMENT 3")
	self.__load_program_and_run__("segment3a-pre.pkl", id)

        rospy.loginfo("STARTING SEGMENT 3")
        rospy.loginfo("10a. Open microwave")
        self.__load_program_and_run__("p2.pkl", id)

        rospy.loginfo("10b. Move microwave lid ")
        self.__load_program_and_run__("p2b.pkl", id)
        # else:
        #     print("Food item " + str(id) + " does not exist.")

    def start_segment3b(self, id):
        """
        (Segment 3)
            11. Grab lunchbox (p6a.pkl, p6b.pkl)
        """
        self.start_obstacles_2()
        rospy.sleep(4)

        rospy.loginfo("11. Grab lunchbox")
        # print('lowering torso for better view')
        # self.torso.set_height(0.35)
        self.__load_program_and_run__("p6a.pkl", id)

        self.attach_lunchbox()
        rospy.sleep(2)

        self.__load_program_and_run__("p6b.pkl", id)

    def start_segment3c(self, id):
        """
        (Segment 3)
            12. (OMITTED) Move to dropoff pose
            13. Put down lunchbox (p7.pkl)
        """
        self.start_obstacles_2()
        rospy.sleep(4)

        # rospy.loginfo("12. Move to dropoff pose")
        # self._map_annotator.goto_position(self.DROPOFF_LOCATION_NAME)
        # rospy.sleep(2)

        rospy.loginfo("13. Put down lunchbox")
        self.__load_program_and_run__("p7.pkl", id)

        self.remove_lunchbox()

        rospy.loginfo("FINISHED SEGMENT 3")


    def start_segment4(self, id):
        """
        (Segment 4)
            14. (OMITTED) Move to start pose
            15. Close microwave (p4a.pkl, p4b.pkl)
        """
        # if id in self._food_items:
        rospy.loginfo("STARTING SEGMENT 4")
        # rospy.loginfo("14. Move to start pose")
        # self._map_annotator.goto_position(self.MICROWAVE_LOCATION_NAME)
        # rospy.sleep(2)
        
        self.start_obstacles_2()
        rospy.sleep(4)

        rospy.loginfo("15a. Close microwave pt. 1")
        self.__load_program_and_run__("p4a.pkl", id)
        rospy.sleep(1.5)

        rospy.loginfo("15b. Changing obstacles...")
        self.start_obstacles_1()
        rospy.sleep(4)

        rospy.loginfo("15b. Close microwave pt. 2")
        self.__load_program_and_run__("p4b.pkl", id)
        rospy.sleep(1.5)

        rospy.loginfo("FINISHED SEGMENT 4")
        # else:
        #     print("Food item " + str(id) + " does not exist.")

    def handle_start_sequence(self, request):
        """
        input: request(id)

        Starts the entire food sequence and removes the food item from the dictionary after it has been finished.

        """
        id = request.id
        # if id in self._food_items:
        rospy.loginfo("Starting sequence for food item: " + str(self._food_items[id]))

        self.start_segment1a(id)
        self.start_segment1b(id)
        self.start_segment1c(id)

        self.start_segment2(id)

        self.start_segment3a(id)
        self.start_segment3b(id)
        self.start_segment3b(id)

        # we decioded that we don't need to close the door :)
        self.start_segment4(id)

        rospy.loginfo("Remove food item from the list.")
        # self.__remove_food_item__(id)

        rospy.loginfo("Finished sequence.")
        # else:
        #     print("Food item " + str(id) + " does not exist.")

        return StartSequenceResponse()


def main():
    rospy.init_node("roboeats_server")
    wait_for_time()

    server = RoboEatsServer()
    create_food_item_service = rospy.Service('roboeats/create_food_item', CreateFoodItem,
                                  server.handle_create_food_item)
    remove_food_item_service = rospy.Service('roboeats/remove_food_item', RemoveFoodItem,
                                             server.handle_remove_food_item)
    start_sequence_service = rospy.Service('roboeats/start_sequence', StartSequence,
                                             server.handle_start_sequence)
    rospy.loginfo("roboeats_server: running...")

    rospy.spin()


if __name__ == '__main__':
  main()
