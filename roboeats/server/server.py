#!/usr/bin/env python

import rospy
import os
import pickle

import robot_api
import tf
import tf.transformations as tft

from geometry_msgs.msg import Pose, PoseWithCovarianceStamped

from map_annotator import Annotator
from pbd import Program, Command

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

class FoodItem(object):
    def __init__(self, name, description, id):
        self.name = name
        self.description = description
        self.id = id
    
    def __str__(self):
        return "FoodItem(name='%s', description='%s', id=%d)" % (self.name, self.description, self.id)

class RoboEatsServer(object):
    DEFAULT_TORSO_HEIGHT = 0.4

    MICROWAVE_LOCATION_NAME = "microwave_location"
    DROPOFF_LOCATION_NAME = "dropoff_location"
    FOOD_ALIAS = "food"

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
                program = pickle.load(load_file)
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


    def handle_start_sequence(self, request):
        """
        input: request(id)

        Starts the entire food sequence and removes the food item from the dictionary after it has been finished.

        Sequence:

        1. Move to start pose
        2. Open microwave
        3. Grab lunchbox
        4. Put it into microwave
        5. Close microwave
        6. Enter time (1 min)
        7. Start microwave
        8. Wait for food to finish microwaving
        9. Wait for cooldown
        10. Open microwave
        11. Grab lunchbox
        12. Move to dropoff pose
        13. Put down lunchbox
        14. Move to start pose
        15. Close microwave
        """
        id = request.id
        rospy.loginfo("Starting sequence for food item: " + str(self._food_items[id]))

        rospy.loginfo("0. Adjust torso height")
        torso = robot_api.Torso()
        torso.set_height(self.DEFAULT_TORSO_HEIGHT)
        rospy.sleep(2)

        rospy.loginfo("1. Move to start pose")
        self._map_annotator.goto_position(self.MICROWAVE_LOCATION_NAME)
        rospy.sleep(2)

        rospy.loginfo("2. Open microwave")
        self.__load_program_and_run__("pbd2.pkl", id)

        rospy.loginfo("3. Grab lunchbox")
        self.__load_program_and_run__("pbd1.pkl", id)

        rospy.loginfo("4. Put it into microwave")
        self.__load_program_and_run__("pbd3.pkl", id)
 
        rospy.loginfo("5. Close microwave")
        self.__load_program_and_run__("pbd4.pkl", id)

        rospy.loginfo("6. Enter time(1 min)")
        self.__load_program_and_run__("pbd5.pkl", id)

        rospy.loginfo("7. Start microwave")
        self.__load_program_and_run__("pbd6.pkl", id)

        rospy.loginfo("8. Wait for food to finish microwaving (in seconds)")
        rospy.sleep(60)

        rospy.loginfo("9. Wait for cooldown (in seconds)")
        rospy.sleep(40) 

        rospy.loginfo("10. Open microwave")
        self.__load_program_and_run__("pbd2.pkl", id)

        rospy.loginfo("11. Grab lunchbox")
        self.__load_program_and_run__("pbd1.pkl", id)

        rospy.loginfo("12. Move to dropoff pose")
        self._map_annotator.goto_position(self.DROPOFF_LOCATION_NAME)
        rospy.sleep(2)

        rospy.loginfo("13. Put down lunchbox")
        self.__load_program_and_run__("pbd7.pkl", id)

        rospy.loginfo("14. Move to start pose")
        self._map_annotator.goto_position(self.MICROWAVE_LOCATION_NAME)
        rospy.sleep(2)

        rospy.loginfo("15. Close microwave")
        self.__load_program_and_run__("pbd4.pkl", id)

        rospy.loginfo("Remove food item from the list.")
        self.__remove_food_item__(id)

        rospy.loginfo("Finished sequence.")

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
