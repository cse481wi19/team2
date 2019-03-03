#!/usr/bin/env python

import rospy
import os
import pickle

from geometry_msgs.msg import Pose, PoseWithCovarianceStamped

from map_annotator import Annotator

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

    MICROWAVE_LOCATION_NAME = "microwave_location"
    DROPOFF_LOCATION_NAME = "dropoff_location"

    def __init__(self, save_file_path="food_items.pkl", nav_file_path="roboeats_nav.pkl"):
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

        # We should expect the nav file given to contain the annotated positions:
        #   MICROWAVE_LOCATION_NAME - starting location in front of the microwave.
        #   DROPOFF_LOCATION_NAME - ending dropoff location.
        self._map_annotator = Annotator(save_file_path=nav_file_path)
        rospy.loginfo("Initialization finished...")

    def __print_food_items__(self):
        rospy.loginfo("Current food items:")
        for f in self._food_items.values():
            rospy.loginfo("\t" + str(f))

    def __save_file__(self):
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

        Removes the food item with the given id if it exists.
        """
        self.__remove_food_item__(request.id)
        return RemoveFoodItemResponse()

    def handle_start_sequence(self, request):
        """
        input: request(id)

        Starts the entire food sequence and removes the food item from the dictionary after it has been finished.
        """
        id = request.id
        print("Starting sequence for food item: " + str(self._food_items[id]))
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
