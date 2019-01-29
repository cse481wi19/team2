#!/usr/bin/env python

import robot_api
import rospy
from web_teleop.srv import SetTorso, SetTorsoResponse
from web_teleop.srv import SetHead, SetHeadResponse
from web_teleop.srv import SetGripper, SetGripperResponse
from web_teleop.srv import SetArm, SetArmResponse


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


class ActuatorServer(object):
    def __init__(self):
        self._torso = robot_api.Torso()
        self._head = robot_api.Head()
        self._gripper = robot_api.Gripper()
        self._arm = robot_api.Arm()
    def handle_set_torso(self, request):
        # TODO: move the torso to the requested height
        self._torso.set_height(request.height)
        return SetTorsoResponse()
    
    def handle_set_head(self, request):
        self._head.pan_tilt(request.pan, request.tilt)
        return SetHeadResponse()
    
    def handle_set_gripper(self, request):
        if request.to_open:
            self._gripper.open()
        else:
            self._gripper.close()
        return SetGripperResponse()

    def handle_set_arm(self, request):
        arm_values = [request.shoulder_pan_joint,
                      request.shoulder_lift_joint,
                      request.upperarm_roll_joint,
                      request.elbow_flex_joint,
                      request.forearm_roll_joint,
                      request.wrist_flex_joint,
                      request.wrist_roll_joint]
        self._arm.move_to_joints(robot_api.ArmJoints.from_list(arm_values))
        return SetArmResponse()

def main():
    rospy.init_node('web_teleop_actuators')
    wait_for_time()
    server = ActuatorServer()
    torso_service = rospy.Service('web_teleop/set_torso', SetTorso,
                                  server.handle_set_torso)
    head_service = rospy.Service('web_teleop/set_head', SetHead,
                                  server.handle_set_head)
    gripper_service = rospy.Service('web_teleop/set_gripper', SetGripper,
                                  server.handle_set_gripper)
    arm_service = rospy.Service('web_teleop/set_arm', SetArm,
                                  server.handle_set_arm)
    rospy.spin()


if __name__ == '__main__':
    main()