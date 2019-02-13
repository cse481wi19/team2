#! /usr/bin/env python

import robot_api
import rospy
import copy

from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import ColorRGBA
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback, MenuEntry
from visualization_msgs.msg import Marker

def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


GRIPPER_MESH = 'package://fetch_description/meshes/gripper_link.dae'
L_FINGER_MESH = 'package://fetch_description/meshes/l_gripper_finger_link.STL'
R_FINGER_MESH = 'package://fetch_description/meshes/r_gripper_finger_link.STL'

ID_GOTO = 1
ID_OPEN = 2
ID_CLOSE = 3

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

    def update_marker(self, is_color_good, pose=None):
        if pose is None:
            pose = Pose()
        
        color = ColorRGBA()
        if is_color_good:
            color.r = 0.5
            color.g = 1
            color.b = 0.5
            color.a = 1
        else:
            color.r = 1
            color.g = 0.25
            color.b = 0.25
            color.a = 1
        
        x_offset = 0.166

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
        gripper_marker.color = color
        gripper_marker.pose.position.x = x_offset

        r_gripper_marker = Marker()
        r_gripper_marker.text = "r_gripper_tip"
        r_gripper_marker.type = Marker.MESH_RESOURCE
        r_gripper_marker.mesh_resource = R_FINGER_MESH
        r_gripper_marker.scale.x = 1
        r_gripper_marker.scale.y = 1
        r_gripper_marker.scale.z = 1
        r_gripper_marker.color = color
        r_gripper_marker.pose.position.y = 0.05
        r_gripper_marker.pose.position.x = x_offset

        l_gripper_marker = Marker()
        l_gripper_marker.text = "l_gripper_tip"
        l_gripper_marker.type = Marker.MESH_RESOURCE
        l_gripper_marker.mesh_resource = L_FINGER_MESH
        l_gripper_marker.scale.x = 1
        l_gripper_marker.scale.y = 1
        l_gripper_marker.scale.z = 1
        l_gripper_marker.color = color
        l_gripper_marker.pose.position.y = -0.05
        l_gripper_marker.pose.position.x = x_offset

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

        goto_me = MenuEntry()
        goto_me.id = ID_GOTO
        goto_me.title = "Gripper Goto Here"
        goto_me.command_type = MenuEntry.FEEDBACK

        open_me = MenuEntry()
        open_me.id = ID_OPEN
        open_me.title = "Gripper Open"
        open_me.command_type = MenuEntry.FEEDBACK

        close_me = MenuEntry()
        close_me.id = ID_CLOSE
        close_me.title = "Gripper Close"
        close_me.command_type = MenuEntry.FEEDBACK

        gripper_im.menu_entries.append(goto_me)
        gripper_im.menu_entries.append(open_me)
        gripper_im.menu_entries.append(close_me)

        self._im_server.insert(gripper_im, feedback_cb=self.handle_feedback)
        self._im_server.applyChanges()

    def start(self):
        ps = PoseStamped()
        ps.header.frame_id = "base_link"
        if self._arm.compute_ik(ps):
            # Found IK
            self.update_marker(True)
        else:
            # Didn't find IK
            self.update_marker(False)

    def handle_feedback(self, feedback):
        if feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
            print(feedback)
            if feedback.menu_entry_id == ID_GOTO:
                print("GOTO PRESSED")
                ps = PoseStamped()
                ps.pose = feedback.pose
                ps.header.frame_id = "base_link"
                self._arm.move_to_pose(ps)
            elif feedback.menu_entry_id == ID_OPEN:
                print("OPEN PRESSED")
                self._gripper.open()
            elif feedback.menu_entry_id == ID_CLOSE:
                print("CLOSE PRESSED")
                self._gripper.close()
        elif feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            ps = PoseStamped()
            ps.pose = feedback.pose
            ps.header.frame_id = "base_link"
            if self._arm.compute_ik(ps):
                # Found IK
                self.update_marker(True, pose=feedback.pose)
            else:
                # Didn't find IK
                self.update_marker(False, pose=feedback.pose)
        else:
            print(feedback)
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
