#!/usr/bin/env python

import rospy

from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from visualization_msgs.msg import Marker


def handle_viz_input(input):
    if (input.event_type == InteractiveMarkerFeedback.BUTTON_CLICK):
        rospy.loginfo(input.marker_name + ' was clicked.')
    elif (input.event_type == InteractiveMarkerFeedback.MOUSE_UP):
        rospy.loginfo(input.marker_name + ' up.')
        
    # else:
    #     rospy.loginfo('Cannot handle this InteractiveMarker event')


class Server(object):
    def __init__(self):
        self._server = InteractiveMarkerServer("simple_marker")
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "base_link"
        int_marker.name = "my_marker"
        int_marker.description = "Simple Click Control"
        int_marker.pose.position.x = 1
        int_marker.pose.orientation.w = 1

        marker = Marker()
        marker.type = Marker.SPHERE
        marker.pose.orientation.w = 1
        marker.scale.x = 0.45
        marker.scale.y = 0.25
        marker.scale.z = 0.25
        marker.color.r = 0.0
        marker.color.g = 0.5
        marker.color.b = 0.5
        marker.color.a = 1.0

        control1 = InteractiveMarkerControl()
        control1.name = "position_control"
        control1.always_visible = True
        control1.markers.append(marker)
        control1.orientation.w = 1
        control1.orientation.x = 0
        control1.orientation.y = 1
        control1.orientation.z = 0
        control1.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        int_marker.controls.append(control1)

        marker2 = Marker()
        marker2.type = Marker.ARROW
        marker2.pose.orientation.w = 1
        marker2.pose.position.z = 0.25
        marker2.scale.x = 0.35
        marker2.scale.y = 0.1
        marker2.scale.z = 0.1
        marker2.color.r = 0.0
        marker2.color.g = 0.5
        marker2.color.b = 0.5
        marker2.color.a = 1.0

        control2 = InteractiveMarkerControl()
        control2.name = "rotation_control"
        control2.always_visible = True
        control2.markers.append(marker2)
        control2.orientation.w = 1
        control2.orientation.x = 0
        control2.orientation.y = 1
        control2.orientation.z = 0
        control2.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(control2)

        self._server.insert(int_marker, self.cb)
        self._server.applyChanges()
        rospy.sleep(3)
    
    def cb(self, input):
        if (input.event_type == InteractiveMarkerFeedback.BUTTON_CLICK):
            rospy.loginfo(input.marker_name + ' was clicked.')
        elif (input.event_type == InteractiveMarkerFeedback.MOUSE_UP):
            rospy.loginfo(input.marker_name + ' up.')

            rospy.loginfo(self._server.get("my_marker").pose)
        # else:
        #     rospy.loginfo('Cannot handle this InteractiveMarker event')


def main():
    rospy.init_node('my_node')
    rospy.sleep(0.5)
    server = Server()
    
    # This is how you erase
    # server.erase("my_marker")
    # server.applyChanges()

    # If we want to do stuff with context menus:
    # https://github.com/ros-visualization/visualization_tutorials/blob/indigo-devel/interactive_marker_tutorials/scripts/menu.py
    rospy.spin()
    
if __name__ == '__main__':
  main()
