#!/usr/bin/env python

import rospy
import copy
import robot_api
import math

from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Odometry


def handle_viz_input(input):
    if (input.event_type == InteractiveMarkerFeedback.BUTTON_CLICK):
        rospy.loginfo(input.marker_name + ' was clicked.')
    else:
        rospy.loginfo('Cannot handle this InteractiveMarker event')


class MarkerTele(object):

    DISTANCE_MAGNITUDE = 0.5
    ANGULAR_MAGNITUDE = math.pi / 6

    def __init__(self):
        self._server = InteractiveMarkerServer("simple_marker")
        self._base = robot_api.Base()

    def click_forward(self, input):
        if (input.event_type == InteractiveMarkerFeedback.BUTTON_CLICK):
            self._base.go_forward(self.DISTANCE_MAGNITUDE)

    def click_backward(self, input):
        if (input.event_type == InteractiveMarkerFeedback.BUTTON_CLICK):
            self._base.go_forward(self.DISTANCE_MAGNITUDE * -1)

    def click_left(self, input):
        if (input.event_type == InteractiveMarkerFeedback.BUTTON_CLICK):
            self._base.turn(self.ANGULAR_MAGNITUDE)
    
    def click_right(self, input):
        if (input.event_type == InteractiveMarkerFeedback.BUTTON_CLICK):
            self._base.turn(self.ANGULAR_MAGNITUDE * -1)

    def callback(self, msg):
        forward_int_marker = InteractiveMarker()
        forward_int_marker.header.frame_id = "base_link"
        forward_int_marker.name = "forward_marker"
        forward_int_marker.description = "Forward"
        forward_int_marker.pose.position.x = 0.5
        forward_int_marker.pose.position.z = 0.75
        forward_int_marker.pose.orientation.w = 0
        forward_marker = Marker()
        forward_marker.ns = "front_marker"
        forward_marker.type = Marker.CUBE
        forward_marker.pose.orientation.w = 0
        forward_marker.scale.x = 0.45
        forward_marker.scale.y = 0.45
        forward_marker.scale.z = 0.45
        forward_marker.color.r = 0.0
        forward_marker.color.g = 0.5
        forward_marker.color.b = 0.5
        forward_marker.color.a = 1.0

        backward_int_marker = InteractiveMarker()
        backward_int_marker.header.frame_id = "base_link"
        backward_int_marker.name = "backward_marker"
        backward_int_marker.description = "Backward"
        backward_int_marker.pose.position.x = -0.5
        backward_int_marker.pose.position.z = 0.75
        backward_int_marker.pose.orientation.w = 0
        backward_marker = Marker()
        backward_marker.ns = "back_marker"
        backward_marker.type = Marker.CUBE
        backward_marker.pose.orientation.w = 0
        backward_marker.scale.x = 0.45
        backward_marker.scale.y = 0.45
        backward_marker.scale.z = 0.45
        backward_marker.color.r = 0.0
        backward_marker.color.g = 0.5
        backward_marker.color.b = 0.5
        backward_marker.color.a = 1.0

        left_int_marker = InteractiveMarker()
        left_int_marker.header.frame_id = "base_link"
        left_int_marker.name = "left_marker"
        left_int_marker.description = "left"
        left_int_marker.pose.position.y = 0.5
        left_int_marker.pose.position.z = 0.75
        left_int_marker.pose.orientation.w = 0
        left_marker = Marker()
        left_marker.ns = "left_marker"
        left_marker.type = Marker.CUBE
        left_marker.pose.orientation.w = 0
        left_marker.scale.x = 0.45
        left_marker.scale.y = 0.45
        left_marker.scale.z = 0.45
        left_marker.color.r = 0.0
        left_marker.color.g = 0.5
        left_marker.color.b = 0.5
        left_marker.color.a = 1.0

        right_int_marker = InteractiveMarker()
        right_int_marker.header.frame_id = "base_link"
        right_int_marker.name = "right_marker"
        right_int_marker.description = "right"
        right_int_marker.pose.position.y = -0.5
        right_int_marker.pose.position.z = 0.75
        right_int_marker.pose.orientation.w = 0
        right_marker = Marker()
        right_marker.ns = "right_marker"
        right_marker.type = Marker.CUBE
        right_marker.pose.orientation.w = 0
        right_marker.scale.x = 0.45
        right_marker.scale.y = 0.45
        right_marker.scale.z = 0.45
        right_marker.color.r = 0.0
        right_marker.color.g = 0.5
        right_marker.color.b = 0.5
        right_marker.color.a = 1.0
        

        button_control = InteractiveMarkerControl()
        button_control.interaction_mode = InteractiveMarkerControl.BUTTON
        button_control.always_visible = True
        button_control.markers.append(forward_marker)
        button_control.markers.append(backward_marker)
        button_control.markers.append(left_marker)
        button_control.markers.append(right_marker)

        forward_int_marker.controls.append(button_control)
        backward_int_marker.controls.append(button_control)
        left_int_marker.controls.append(button_control)
        right_int_marker.controls.append(button_control)

        self._server.insert(forward_int_marker, self.click_forward)
        self._server.insert(backward_int_marker, self.click_backward)
        self._server.insert(left_int_marker, self.click_left)
        self._server.insert(right_int_marker, self.click_right)

        self._server.applyChanges()



def main():
    rospy.init_node('marker_teleop')
    rospy.sleep(0.5)

    tele = MarkerTele()

    rospy.Subscriber('odom', Odometry, tele.callback)
    rospy.spin()

    
if __name__ == '__main__':
  main()