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

        rotation_ring_control = InteractiveMarkerControl()
        rotation_ring_control.name = "position_control"
        rotation_ring_control.always_visible = True
        rotation_ring_control.orientation.w = 1
        rotation_ring_control.orientation.x = 0
        rotation_ring_control.orientation.y = 1
        rotation_ring_control.orientation.z = 0
        rotation_ring_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(rotation_ring_control)

        arrow_marker = Marker()
        arrow_marker.type = Marker.ARROW
        arrow_marker.pose.orientation.w = 1
        arrow_marker.pose.position.z = 0.25
        arrow_marker.scale.x = 0.6
        arrow_marker.scale.y = 0.15
        arrow_marker.scale.z = 0.15
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
