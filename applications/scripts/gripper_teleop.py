#! /usr/bin/env python

import robot_api
import rospy
import copy
import itertools

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


def make_6dof_controls():
    controls = []
    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 1
    control.orientation.y = 0
    control.orientation.z = 0
    control.name = "rotate_x"
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    controls.append(copy.deepcopy(control))

    control.name = "move_x"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    controls.append(copy.deepcopy(control))

    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 1
    control.orientation.z = 0
    control.name = "rotate_z"
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    controls.append(copy.deepcopy(control))

    control.name = "move_z"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    controls.append(copy.deepcopy(control))

    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 0
    control.orientation.z = 1
    control.name = "rotate_y"
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    controls.append(copy.deepcopy(control))

    control.name = "move_y"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    controls.append(copy.deepcopy(control))
    return controls

class GripperTeleop(object):

    ID_GOTO = 1
    ID_OPEN = 2
    ID_CLOSE = 3

    def __init__(self, arm, gripper, im_server):
        self._arm = arm
        self._gripper = gripper
        self._im_server = im_server

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
        gripper_im.controls.extend(make_6dof_controls())

        goto_me = MenuEntry()
        goto_me.id = self.ID_GOTO
        goto_me.title = "Gripper Goto Here"
        goto_me.command_type = MenuEntry.FEEDBACK

        open_me = MenuEntry()
        open_me.id = self.ID_OPEN
        open_me.title = "Gripper Open"
        open_me.command_type = MenuEntry.FEEDBACK

        close_me = MenuEntry()
        close_me.id = self.ID_CLOSE
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


class AutoPickTeleop(object):
    ID_PICKFRONT = 1
    ID_OPEN = 2

    STAGE_PREFIXES = ["_s1", "_s2", "_s2"]
    X_OFFSETS = [-0.1, 0.166, 0.166]
    Y_OFFSETS = [0, 0, 0]
    Z_OFFSETS = [0, 0, 0.125]

    def __init__(self, arm, gripper, im_server):
        self._arm = arm
        self._gripper = gripper
        self._im_server = im_server

    def create_gripper_markers(self, pose, prefix="", xoffset=0, yoffset=0, zoffset=0):
        markers = []

        ps = PoseStamped()
        ps.pose = pose
        ps.pose.position.x += xoffset
        ps.pose.position.y += yoffset
        ps.pose.position.z += zoffset
        ps.header.frame_id = "base_link"
        
        color = ColorRGBA()
        if self._arm.compute_ik(ps, verbose=False):
            color.r = 0.5
            color.g = 1
            color.b = 0.5
            color.a = 1
        else:
            color.r = 1
            color.g = 0.25
            color.b = 0.25
            color.a = 1


        wrist_link_xoff = 0.166

        gripper_marker = Marker()
        gripper_marker.text = "gripper_link" + prefix
        gripper_marker.type = Marker.MESH_RESOURCE
        gripper_marker.mesh_resource = GRIPPER_MESH
        gripper_marker.scale.x = 1
        gripper_marker.scale.y = 1
        gripper_marker.scale.z = 1
        gripper_marker.color = color
        gripper_marker.pose.position.x = xoffset + wrist_link_xoff
        gripper_marker.pose.position.y = yoffset
        gripper_marker.pose.position.z = zoffset

        r_gripper_marker = Marker()
        r_gripper_marker.text = "r_gripper_tip" + prefix
        r_gripper_marker.type = Marker.MESH_RESOURCE
        r_gripper_marker.mesh_resource = R_FINGER_MESH
        r_gripper_marker.scale.x = 1
        r_gripper_marker.scale.y = 1
        r_gripper_marker.scale.z = 1
        r_gripper_marker.color = color
        r_gripper_marker.pose.position.x = xoffset + wrist_link_xoff
        r_gripper_marker.pose.position.y = 0.05 + yoffset
        r_gripper_marker.pose.position.z = zoffset

        l_gripper_marker = Marker()
        l_gripper_marker.text = "l_gripper_tip" + prefix
        l_gripper_marker.type = Marker.MESH_RESOURCE
        l_gripper_marker.mesh_resource = L_FINGER_MESH
        l_gripper_marker.scale.x = 1
        l_gripper_marker.scale.y = 1
        l_gripper_marker.scale.z = 1
        l_gripper_marker.color = color
        l_gripper_marker.pose.position.x = xoffset + wrist_link_xoff
        l_gripper_marker.pose.position.y = -0.05 + yoffset
        l_gripper_marker.pose.position.z = zoffset

        markers.append(gripper_marker)
        markers.append(r_gripper_marker)
        markers.append(l_gripper_marker)
        return markers

    def update_marker(self, pose=None):
        if pose is None:
            pose = Pose()

        gripper_im = InteractiveMarker()
        gripper_im.header.frame_id = "base_link"
        gripper_im.name = "gripper_int"
        gripper_im.description = "gripper_int"

        # Setup context menu options
        pickfront_me = MenuEntry()
        pickfront_me.id = self.ID_PICKFRONT
        pickfront_me.title = "Pick from front"
        pickfront_me.command_type = MenuEntry.FEEDBACK

        open_me = MenuEntry()
        open_me.id = self.ID_OPEN
        open_me.title = "Gripper open"
        open_me.command_type = MenuEntry.FEEDBACK

        gripper_im.menu_entries.append(pickfront_me)
        gripper_im.menu_entries.append(open_me)

        gripper_im.pose = pose
        gripper_im.controls.extend(make_6dof_controls())

        gripper_control = InteractiveMarkerControl()
        gripper_control.name = "gripper_link_control"
        gripper_control.always_visible = True
        gripper_control.interaction_mode = InteractiveMarkerControl.NONE
        
        for xoff, yoff, zoff, pref in itertools.izip(self.X_OFFSETS, self.Y_OFFSETS, self.Z_OFFSETS, self.STAGE_PREFIXES):
            gripper_control.markers.extend(
                self.create_gripper_markers(copy.deepcopy(pose), prefix=pref, xoffset=xoff, yoffset=yoff, zoffset=zoff))

        gripper_im.controls.append(gripper_control)

        self._im_server.insert(
            gripper_im, feedback_cb=self.handle_feedback)
        self._im_server.applyChanges()


    def start(self):
        ps = PoseStamped()
        ps.header.frame_id = "base_link"
        if self._arm.compute_ik(ps):
            # Found IK
            self.update_marker()
        else:
            # Didn't find IK
            self.update_marker()

    def handle_feedback(self, feedback):
        if feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
            print(feedback)
            if feedback.menu_entry_id == self.ID_PICKFRONT:
                print("START SEQUENCE")
                ps = PoseStamped()
                self._gripper.open()
                for i, tup in enumerate(itertools.izip(self.X_OFFSETS, self.Y_OFFSETS, self.Z_OFFSETS)):
                    xoff, yoff, zoff = tup
                    ps.pose = copy.deepcopy(feedback.pose)
                    ps.pose.position.x += xoff
                    ps.pose.position.y += yoff
                    ps.pose.position.z += zoff
                    ps.header.frame_id = "base_link"
                    self._arm.move_to_pose(ps)
                    if i == 1:
                        self._gripper.close()
            elif feedback.menu_entry_id == self.ID_OPEN:
                print("OPEN PRESSED")
                self._gripper.open()
        elif feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            print(feedback)
            self.update_marker(pose=feedback.pose)


def main():
    rospy.init_node('gripper_teleop')
    rospy.sleep(0.5)
    wait_for_time()
    argv = rospy.myargv()

    arm = robot_api.Arm()
    gripper = robot_api.Gripper()

    # im_server = InteractiveMarkerServer('gripper_im_server') 
    # teleop = GripperTeleop(arm, gripper, im_server)
    # teleop.start()

    auto_pick_im_server = InteractiveMarkerServer('auto_pick_im_server')
    auto_pick = AutoPickTeleop(arm, gripper, auto_pick_im_server)
    auto_pick.start()

    rospy.spin()
    

if __name__ == '__main__':
    main()
