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
    rospy.init_node('part2_obstacles')
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

    rospy.sleep(2)
    rospy.spin()


if __name__ == '__main__':
    main()
