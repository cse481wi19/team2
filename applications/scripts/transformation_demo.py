#! /usr/bin/env python

import math
import numpy as np
from geometry_msgs.msg import Quaternion, Point, Pose, PoseStamped, Vector3
from std_msgs.msg import ColorRGBA
import visualization_msgs.msg
import rospy
import tf.transformations as tft
from visualization_msgs.msg import Marker
from std_msgs.msg import Header, ColorRGBA


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


def cosd(degs):
    return math.cos(degs * math.pi / 180)


def sind(degs):
    return math.sin(degs * math.pi / 180)


def axis_marker(pose_stamped):
    marker = visualization_msgs.msg.Marker()
    marker.ns = 'axes'
    marker.header = pose_stamped.header
    marker.header.stamp = rospy.Time.now()
    marker.pose = pose_stamped.pose
    marker.type = visualization_msgs.msg.Marker.LINE_LIST
    marker.scale.x = 0.1

    marker.points.append(Point(0, 0, 0))
    marker.colors.append(ColorRGBA(1, 0, 0, 1))
    marker.points.append(Point(1, 0, 0))
    marker.colors.append(ColorRGBA(1, 0, 0, 1))

    marker.points.append(Point(0, 0, 0))
    marker.colors.append(ColorRGBA(0, 1, 0, 1))
    marker.points.append(Point(0, 1, 0))
    marker.colors.append(ColorRGBA(0, 1, 0, 1))

    marker.points.append(Point(0, 0, 0))
    marker.colors.append(ColorRGBA(0, 0, 1, 1))
    marker.points.append(Point(0, 0, 1))
    marker.colors.append(ColorRGBA(0, 0, 1, 1))

    return marker


def transform_to_pose(matrix):
    pose = Pose()
    print("matrix:", matrix)
    # TODO: fill this out
    print(tft.quaternion_from_matrix(matrix))
    frame_b_point = np.matrix([[1], [0], [0], [1]])
    print(frame_b_point)
    # print(np.multiply(matrix, frame_b_point))
    ans = matrix * frame_b_point
    print(ans)

    print(tft.quaternion_matrix(
        [ans.item(0), ans.item(1), 0, 0]))

    print(tft.quaternion_from_matrix(matrix))
    ans2 = tft.quaternion_from_matrix(matrix)

    pose.orientation.x = ans2[0]
    pose.orientation.y = ans2[1]
    pose.orientation.z = ans2[2]
    pose.orientation.w = ans2[3]
    pose.position.z = ans.item(2)

    return pose


def arrow_marker(point):
    marker = visualization_msgs.msg.Marker()
    marker.ns = 'arrow'
    marker.type = visualization_msgs.msg.Marker.ARROW
    marker.header.frame_id = 'frame_a'
    marker.points.append(Point(0, 0, 0))
    marker.points.append(point)
    marker.scale.x = 0.1
    marker.scale.y = 0.15
    marker.color.r = 1
    marker.color.g = 1
    marker.color.a = 1
    return marker


def main():
    rospy.init_node('transformation_demo')
    wait_for_time()
    viz_pub = rospy.Publisher(
        'visualization_marker', visualization_msgs.msg.Marker, queue_size=1)
    rospy.sleep(0.5)
    b_in_a = np.array([
        [cosd(45), -sind(45), 0, 0],
        [sind(45), cosd(45), 0, 0],
        [0, 0, 1, 0.5],
        [0, 0, 0, 1]
    ])
    ps = PoseStamped()
    ps.header.frame_id = 'frame_a'
    ps.pose = transform_to_pose(b_in_a)
    viz_pub.publish(axis_marker(ps))

    point_in_b = np.array([1, 0, 0, 1])
    point_in_a = np.dot(b_in_a, point_in_b)
    rospy.loginfo(point_in_b)
    rospy.loginfo(point_in_a)
    point = Point(point_in_a[0], point_in_a[1], point_in_a[2])
    viz_pub.publish(arrow_marker(point))

    # base_P = base_T_object * object_P

    base_T_object_matrix = tft.quaternion_matrix(
        [0, 0, 0.38268343, 0.92387953])
    base_T_object_matrix[0, 3] = 0.6
    base_T_object_matrix[1, 3] = -0.1
    base_T_object_matrix[2, 3] = 0.7
    print(base_T_object_matrix)

    object_P_matrix = np.array([
        [1, 0, 0, -0.1],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])
    print(object_P_matrix)

    # ans = base_T_object_matrix * object_P_matrix
    ans = np.dot(base_T_object_matrix, object_P_matrix)

    ans2 = tft.quaternion_from_matrix(ans)

    print(ans)
    print(ans2)

    # Answer from dot(base_T_object_matrix, object_P_matrix)
    final_pose = Pose()
    final_pose.position.x = ans[0, 3]
    final_pose.position.y = ans[1, 3]
    final_pose.position.z = ans[2, 3]
    final_pose.orientation = ans2

    rospy.spin()


if __name__ == '__main__':
    main()
