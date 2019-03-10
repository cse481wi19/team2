#! /usr/bin/env python

from moveit_python import PlanningSceneInterface
import robot_api
import rospy


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass



def main():
    rospy.init_node('part1_obstacles')
    wait_for_time()

    planning_scene = PlanningSceneInterface('base_link')
    planning_scene.clear()
    planning_scene.removeCollisionObject('table')
    planning_scene.removeCollisionObject('floor')
    planning_scene.addBox('floor', 2, 2, 0.01, 0, 0, 0.01/2)
    table_height = 0.767
    table_width = 0.7
    table_x = 0.95
    planning_scene.addBox('table', table_width, 2, table_height, table_x, 0, table_height/2)
    planning_scene.addBox('robot_base', 0.54, 0.52, 0.37, 0, 0, 0.37/2)

    microwave_height = 0.28
    microwave_width = 0.48
    microwave_depth = 0.33
    microwave_x = 0.97
    microwave_y = 0.18

    microwave_side_height = 0.2
    microwave_r_width = 0.135
    microwave_r_y = microwave_y - 0.175
    microwave_l_width = 0.035
    microwave_l_y = microwave_y + 0.222
    microwave_bottom_height = 0.05
    microwave_top_height = 0.04
    microwave_back_depth = 0.03
    microwave_back_x = table_x + (microwave_depth / 2) + (microwave_back_depth/2)
    microwave_door_width = 0.09
    microwave_door_x = microwave_x - 0.36
    microwave_door_y = microwave_l_y + 0.027


    planning_scene.addBox('microwave_top', microwave_depth, microwave_width, microwave_top_height, microwave_x, microwave_y, table_height + microwave_bottom_height + microwave_side_height + (microwave_top_height/2))
    planning_scene.addBox('microwave_bottom', microwave_depth, microwave_width, microwave_bottom_height, microwave_x, microwave_y, table_height + (microwave_bottom_height/2))
    planning_scene.addBox('microwave_side_r', microwave_depth, microwave_r_width, microwave_side_height, microwave_x, microwave_r_y, table_height + microwave_bottom_height + (microwave_side_height/2))
    planning_scene.addBox('microwave_side_l', microwave_depth, microwave_l_width, microwave_side_height, microwave_x, microwave_l_y, table_height + microwave_bottom_height +  microwave_side_height/2)
    planning_scene.addBox('microwave_back', microwave_back_depth, microwave_width, microwave_height, microwave_back_x, microwave_y, table_height + microwave_height/2)
    planning_scene.addBox('microwave_door', 0.39, microwave_door_width, microwave_height + 0.01, microwave_door_x, microwave_door_y, table_height + microwave_height/2 + 0.005)

    rospy.sleep(2)
    rospy.spin()


if __name__ == '__main__':
    main()
