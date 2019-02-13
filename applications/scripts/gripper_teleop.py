#! /usr/bin/env python

import robot_api
import rospy
import copy

from geometry_msgs.msg import Pose
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from visualization_msgs.msg import Marker

def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


GRIPPER_MESH = 'package://fetch_description/meshes/gripper_link.dae'
L_FINGER_MESH = 'package://fetch_description/meshes/l_gripper_finger_link.STL'
R_FINGER_MESH = 'package://fetch_description/meshes/r_gripper_finger_link.STL'

class GripperTeleop(object):
    def __init__(self, arm, gripper, im_server):
        self._arm = arm
        self._gripper = gripper
        self._im_server = im_server

    def make_6dof_controls(self):
        controls = []
        control = InteractiveMarkerControl()
        control.orientation.w = 1;
        control.orientation.x = 1;
        control.orientation.y = 0;
        control.orientation.z = 0;
        control.name = "rotate_x";
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS;
        controls.append(copy.deepcopy(control))

        control.name = "move_x";
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS;
        controls.append(copy.deepcopy(control))
         
        control.orientation.w = 1;
        control.orientation.x = 0;
        control.orientation.y = 1;
        control.orientation.z = 0;
        control.name = "rotate_z";
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS;
        controls.append(copy.deepcopy(control))

        control.name = "move_z";
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS;
        controls.append(copy.deepcopy(control))
         
        control.orientation.w = 1;
        control.orientation.x = 0;
        control.orientation.y = 0;
        control.orientation.z = 1;
        control.name = "rotate_y";
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS;
        controls.append(copy.deepcopy(control))

        control.name = "move_y";
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS;
        controls.append(copy.deepcopy(control))
        return controls

    def start(self):
        pose = Pose()
        pose.position.x = 0
        pose.position.y = 0
        pose.position.z = 0

        gripper_im = InteractiveMarker()
        gripper_im.header.frame_id = "base_link"
        gripper_im.name = "gripper_int"
        gripper_im.description = "gripper_int"
        gripper_im.pose = pose

        gripper_marker = Marker()
        gripper_marker.text = "gripper_link"
        gripper_marker.type = Marker.MESH_RESOURCE
        gripper_marker.mesh_resource = GRIPPER_MESH
        gripper_marker.scale.x = 1
        gripper_marker.scale.y = 1
        gripper_marker.scale.z = 1
        gripper_marker.color.r = 0.5
        gripper_marker.color.g = 0.5
        gripper_marker.color.b = 0.5
        gripper_marker.color.a = 1
        gripper_marker.pose.position.x = 0.0166

        r_gripper_marker = Marker()
        r_gripper_marker.text = "r_gripper_tip"
        r_gripper_marker.type = Marker.MESH_RESOURCE
        r_gripper_marker.mesh_resource = R_FINGER_MESH
        r_gripper_marker.scale.x = 1
        r_gripper_marker.scale.y = 1
        r_gripper_marker.scale.z = 1
        r_gripper_marker.color.r = 0.5
        r_gripper_marker.color.g = 0.5
        r_gripper_marker.color.b = 0.5
        r_gripper_marker.color.a = 1
        r_gripper_marker.pose.position.y = 0.05
        r_gripper_marker.pose.position.x = 0.0166

        l_gripper_marker = Marker()
        l_gripper_marker.text = "l_gripper_tip"
        l_gripper_marker.type = Marker.MESH_RESOURCE
        l_gripper_marker.mesh_resource = L_FINGER_MESH
        l_gripper_marker.scale.x = 1
        l_gripper_marker.scale.y = 1
        l_gripper_marker.scale.z = 1
        l_gripper_marker.color.r = 0.5
        l_gripper_marker.color.g = 0.5
        l_gripper_marker.color.b = 0.5
        l_gripper_marker.color.a = 1
        l_gripper_marker.pose.position.y = -0.05
        l_gripper_marker.pose.position.x = 0.0166

        gripper_control = InteractiveMarkerControl()
        gripper_control.name = "gripper_link_control"
        gripper_control.markers.append(gripper_marker)
        gripper_control.markers.append(r_gripper_marker)
        gripper_control.markers.append(l_gripper_marker)
        gripper_control.markers.append(gripper_marker)
        gripper_control.always_visible = True
        gripper_control.interaction_mode = InteractiveMarkerControl.NONE

        gripper_im.controls.append(gripper_control)
        gripper_im.controls.extend(self.make_6dof_controls())

        self._im_server.insert(gripper_im, feedback_cb=self.handle_feedback)
        self._im_server.applyChanges()

    def handle_feedback(self, feedback):
        pass


class AutoPickTeleop(object):
    def __init__(self, arm, gripper, im_server):
        self._arm = arm
        self._gripper = gripper
        self._im_server = im_server

    def start(self):
        # obj_im = InteractiveMarker() ...
        # self._im_server.insert(obj_im, feedback_cb=self.handle_feedback)
        pass

    def handle_feedback(self, feedback):
        pass


def main():
    rospy.init_node('gripper_teleop')
    rospy.sleep(0.5)
    wait_for_time()
    argv = rospy.myargv()

    arm = robot_api.Arm()
    gripper = robot_api.Gripper()

    im_server = InteractiveMarkerServer('gripper_im_server')
    # auto_pick_im_server = InteractiveMarkerServer('auto_pick_im_server')

    teleop = GripperTeleop(arm, gripper, im_server)
    # auto_pick = AutoPickTeleop(arm, gripper, auto_pick_im_server)

    teleop.start()
    # auto_pick.start()

    rospy.spin()
    

if __name__ == '__main__':
    main()
