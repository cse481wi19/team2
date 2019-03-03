#!/usr/bin/env python

import rospy

from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
from roboeats.msg import FoodItems
from roboeats.srv import CreateFoodItem, CreateFoodItemResponse


def wait_for_time():
    """Wait for simulated time to begin.                          
    """
    while rospy.Time().now().to_sec() == 0:
        pass


class RoboEatsServer(object):

    def __init__(self):
        self._pose_names_pub = rospy.Publisher("/roboeats/food_items",
                                               FoodItems, queue_size=10, latch=True)
        print("Initialization finished...")

    def handle_create_food_item(self, request):
        rospy.loginfo(request)


def main():
    rospy.init_node("roboeats_server")
    wait_for_time()
    server = RoboEatsServer()
    create_food_item_service = rospy.Service('roboeats/create_food_item', CreateFoodItem,
                                  server.handle_create_food_item)
    rospy.loginfo("roboeats_server: running...")
    rospy.spin()


if __name__ == '__main__':
  main()
