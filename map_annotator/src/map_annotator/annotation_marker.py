#!/usr/bin/env python

import rospy
import copy
import robot_api
import math

from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Odometry

class CircleMarker(object):
    def __init__(self):
        self._server = InteractiveMarkerServer("simple_marker")

    def callback_marker(self, input):
        pass
    
    def align_marker(self, feedback):
        pose = feedback.pose
        self._server.setPose(feedback.marker_name, pose)
        print("Arrow Marker's name is ", feedback.marker_name)

        self._server.setPose("orientation_ring", pose)
        self._server.applyChanges()

    def orientation_ring_callback(self, feedback):
        if (feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE):
            print("Updating arrow marker's position ")
            self._server.setPose("int_arrow_marker", feedback.pose)
            self._server.applyChanges()

    def create_draggable_marker(self, position):

        int_arrow_marker = InteractiveMarker()
        int_arrow_marker.header.frame_id = "map"
        int_arrow_marker.name = "int_arrow_marker"
        int_arrow_marker.description = "right"
        int_arrow_marker.pose.position.y = position.y
        int_arrow_marker.pose.position.x = position.x
        int_arrow_marker.pose.position.z = position.z + 0.1
        int_arrow_marker.scale = 1

        arrow_marker = Marker()
        arrow_marker.ns = "arrow_marker"
        arrow_marker.type = Marker.ARROW
        arrow_marker.pose.orientation.w = 0
        arrow_marker.scale.x = 0.5
        arrow_marker.scale.y = 0.05
        arrow_marker.scale.z = 0.05
        arrow_marker.color.r = 0.0
        arrow_marker.color.g = 0.5
        arrow_marker.color.b = 0.5
        arrow_marker.color.a = 1.0

        button_control = InteractiveMarkerControl()
        button_control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        button_control.always_visible = True

        button_control.orientation.w = 1
        button_control.orientation.x = 0
        button_control.orientation.y = 1
        button_control.orientation.z = 0
        button_control.markers.append(arrow_marker)
        int_arrow_marker.controls.append(button_control)

        orientation_ring_marker = InteractiveMarker()
        orientation_ring_marker.header.frame_id = "map"
        orientation_ring_marker.scale = 1
        orientation_ring_marker.pose.position.y = position.y
        orientation_ring_marker.pose.position.z = position.z + 0.1
        orientation_ring_marker.pose.position.x = position.x
        orientation_ring_marker.name = "orientation_ring"
        orientation_ring_marker.description = "Orientation Ring"

        orientation_ring_marker_control = InteractiveMarkerControl()
        orientation_ring_marker_control.always_visible = True
        orientation_ring_marker_control.orientation.w = 1
        orientation_ring_marker_control.orientation.x = 0
        orientation_ring_marker_control.orientation.y = 1
        orientation_ring_marker_control.orientation.z = 0

        orientation_ring_marker_control.interaction_mode = InteractiveMarkerControl.MOVE_ROTATE
        orientation_ring_marker.controls.append(orientation_ring_marker_control)
        self._server.insert(orientation_ring_marker, self.orientation_ring_callback)




        self._server.insert(int_arrow_marker, self.callback_marker)
        print"inserted"
        self._server.setCallback("int_arrow_marker", self.align_marker, InteractiveMarkerFeedback.POSE_UPDATE)
        self._server.applyChanges()

