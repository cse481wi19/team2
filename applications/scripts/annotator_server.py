#!/usr/bin/env python

import rospy

from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from visualization_msgs.msg import Marker

from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
from map_annotator import Annotator
from map_annotator.msg import PoseNames
from map_annotator.msg import UserAction


def wait_for_time():
    """Wait for simulated time to begin.                          
    """
    while rospy.Time().now().to_sec() == 0:
        pass

class AnnotatorServer(object):


    def __init__(self):
        self._annotator = Annotator()
        self._pose_names_pub = rospy.Publisher("/map_annotator/pose_names",
                                               PoseNames, queue_size=10, latch=True)
        # self._map_poses_pub = rospy.Publisher("/map_annotator/map_poses",
        #                                        InteractiveMarker, queue_size=10, latch=True)
        self._int_marker_server = InteractiveMarkerServer("/map_annotator/map_poses")

        self.INITIAL_POSE = Pose()
        self.INITIAL_POSE.orientation.w = 1

        print("Initializing saved markers: " +
              str(self._annotator.get_position_names()))
        for name, pose in self._annotator.get_position_items():
            self.__create_int_marker__(name, pose)
        
        self.__pub_pose_info__()
        print("Initialization finished...")

    def __pub_pose_info__(self):
        pose_names = PoseNames()
        pose_names.names = self._annotator.get_position_names()
        pose_names.names.sort()
        self._pose_names_pub.publish(pose_names)

    def __update_marker_pose__(self, input):
        if (input.event_type == InteractiveMarkerFeedback.MOUSE_UP):
            name = input.marker_name
            new_pose = self._int_marker_server.get(name).pose
            # Overwrite the previous pose with the new pose
            self._annotator.save_position(name, new_pose)
            self._int_marker_server.setPose(name, new_pose)
            self._int_marker_server.applyChanges()
            print("updated pose for: " + name)

    def __create_int_marker__(self, name, pose):
        print("creating int marker: " + name)
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "map"
        int_marker.name = name
        int_marker.description = name
        int_marker.pose = pose
        # Move it 0.25 meters up to make it easier to click
        int_marker.pose.position.z = 0.25

        text_marker = Marker()
        text_marker.text = name
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.pose.position.z = 2
        text_marker.scale.x = 0.4
        text_marker.scale.y = 0.4
        text_marker.scale.z = 0.4
        text_marker.color.r = 0.0
        text_marker.color.g = 0.5
        text_marker.color.b = 0.5
        text_marker.color.a = 1.0

        text_control = InteractiveMarkerControl()
        text_control.name = "text_control"
        text_control.markers.append(text_marker)
        text_control.always_visible = True
        text_control.interaction_mode = InteractiveMarkerControl.NONE
        int_marker.controls.append(text_control)

        rotation_ring_control = InteractiveMarkerControl()
        rotation_ring_control.name = "position_control"
        rotation_ring_control.always_visible = True
        rotation_ring_control.orientation.x = 0
        rotation_ring_control.orientation.w = 1
        rotation_ring_control.orientation.y = 1
        rotation_ring_control.orientation.z = 0
        rotation_ring_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(rotation_ring_control)

        arrow_marker = Marker()
        arrow_marker.type = Marker.ARROW
        arrow_marker.pose.orientation.w = 1
        arrow_marker.pose.position.z = 0.15
        arrow_marker.pose.position.x = -0.5
        arrow_marker.scale.x = 1
        arrow_marker.scale.y = 0.25
        arrow_marker.scale.z = 0.25
        arrow_marker.color.r = 0.0
        arrow_marker.color.g = 0.5
        arrow_marker.color.b = 0.5
        arrow_marker.color.a = 1.0

        position_control = InteractiveMarkerControl()
        position_control.name = "rotation_control"
        position_control.always_visible = True
        position_control.markers.append(arrow_marker)
        position_control.orientation.w = 1
        position_control.orientation.x = 0
        position_control.orientation.y = 1
        position_control.orientation.z = 0
        position_control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        int_marker.controls.append(position_control)

        self._int_marker_server.insert(int_marker, self.__update_marker_pose__)
        self._int_marker_server.applyChanges()

    def create(self, name, pose=None):
        print("creating new pose: " + name)
        if pose is None:
            pose = self.INITIAL_POSE
        self._annotator.save_position(name, pose)
        self.__create_int_marker__(name, pose)
        self.__pub_pose_info__()
    
    def delete(self, name):
        if self._annotator.exists(name):
            print("deleting pose: " + name)
            self._annotator.delete_position(name)
            self._int_marker_server.erase(name)
            self._int_marker_server.applyChanges()
            self.__pub_pose_info__()

    def handle_callback(self, user_action_msg):
        cmd = user_action_msg.command
        name = user_action_msg.name
        if cmd == UserAction.CREATE:
            self.create(name)
        elif cmd == UserAction.DELETE:
            self.delete(name)
        elif cmd == UserAction.GOTO:
            print("going to pose: " + name)
            self._annotator.goto_position(name)
        

def main():
    rospy.init_node("annotator_server")
    wait_for_time()
    server = AnnotatorServer()
    rospy.Subscriber("/map_annotator/user_actions",
                     UserAction, server.handle_callback)
    rospy.spin()
 


if __name__ == '__main__':
  main()
