import rospy
import copy
import pickle
import robot_api
import tf
import tf.transformations as tft
import numpy as np

from geometry_msgs.msg import PoseStamped
from command import Command

def ps_str(ps):
    pos = ps.pose.position
    rot = ps.pose.orientation
    return "PoseStamped(pos: (%.3f, %.3f, %.3f), rot: (%.3f, %.3f, %.3f, %.3f), frame_id: %s)" % (pos.x, pos.y, pos.z, rot.x, rot.y, rot.z, rot.w, ps.header.frame_id)


def pos_rot_str(pos, rot):
    return "pos: (%.2f, %.2f, %.2f), rot: (%.2f, %.2f, %.2f, %.2f)" % (pos[0], pos[1], pos[2], rot[0], rot[1], rot[2], rot[3])

class Program(object):
    def __init__(self, arm, gripper):
        self.commands = []
        self.arm = arm
        self.gripper = gripper

    def reset(self):
        self.commands = []

    def add_pose_command(self, ps, alias):
        self.commands.append(
            Command(Command.POSE, pose_stamped=ps, alias=alias))

    def add_open_gripper_command(self):
        self.commands.append(Command(Command.OPEN_GRIPPER))

    def add_close_gripper_command(self):
        self.commands.append(Command(Command.CLOSE_GRIPPER))

    def add_set_height_command(self, height):
        self.commands.append(Command(Command.SET_HEIGHT, height=height))

    def add_set_pan_tilt_command(self, pan, tilt):
        self.commands.append(Command(Command.SET_PAN_TILT, pan=pan, tilt=tilt))
    
    def delete_last_command(self):
        if len(self.commands) > 0:
            self.commands.pop()

    def save_program(self, fp):
        with open(fp, "wb") as save_file:
            pickle.dump(self.commands, save_file)

    def print_program(self):
        for i, command in enumerate(self.commands):
            print(i, str(command))

    def replace_frame(self, alias, new_frame):
        for i, command in enumerate(self.commands):
            if command.alias == alias:
                old_frame = command.pose_stamped.header.frame_id
                command.pose_stamped.header.frame_id = new_frame
                print("Command %d: Replaced '%s' with '%s'" %
                      (i, old_frame, new_frame))
        print("New program:")
        self.print_program()
    def wiggle_head(self):
	head = robot_api.Head()
	head.pan_tilt(-0.1, 0)
	rospy.sleep(2)
	head.pan_tilt(-0.1, 0.57)	

    def run(self, listener=None):
        if listener is None:
            listener = tf.TransformListener()
        try:
            rospy.sleep(1)
            for i, command in enumerate(self.commands):
                print(i, str(command))
                if command.type == Command.POSE:
                    print(ps_str(command.pose_stamped))
                    # ps = copy.deepcopy(command.pose_stamped)
                    ps = command.pose_stamped
                    frame = command.pose_stamped.header.frame_id

                    #############################################################
                    # Retries using move_to_pose only
                    # # Attempt to move to the pose 3 times
                    for i in range(3):
                        print("In program.py retry loop, i=" + str(i))
                        if frame != "base_link":
                            # while True:
                            listener.waitForTransform(
                                "base_link", frame, rospy.Time(), rospy.Duration(4.0))
                            while not rospy.is_shutdown():
                                try:
                                    # now = rospy.Time.now()
                                    now = rospy.Time(0)
                                    listener.waitForTransform(
                                        "base_link", frame, now, rospy.Duration(4.0))
                                    pos, rot = listener.lookupTransform(
                                        "base_link", frame, now)
                                    print("Got transform: " + pos_rot_str(pos, rot))
                                    break
                                except Exception as e:
                                    print("Exception: " + str(e))
                                    pass
                                # x, y, z, w = rot
                                # # if frame == "ar_marker_17" and x < 0 and y < 0 and z > 0 and w < 0:
                                # #                             #    x > 0 and y > 0 and z < 0 and w > 0:
                                # #     break
                                # if frame == "ar_marker_14" and x > 0 and y > 0 and z > 0 and w < 0:
                                #     break
                            base_T_frame_matrix = tft.quaternion_matrix(rot)
                            base_T_frame_matrix[0, 3] = pos[0]
                            base_T_frame_matrix[1, 3] = pos[1]
                            base_T_frame_matrix[2, 3] = pos[2]

                            ori = ps.pose.orientation
                            frame_T_gripper_matrix = tft.quaternion_matrix(
                                [ori.x, ori.y, ori.z, ori.w])
                            frame_T_gripper_matrix[0, 3] = ps.pose.position.x
                            frame_T_gripper_matrix[1, 3] = ps.pose.position.y
                            frame_T_gripper_matrix[2, 3] = ps.pose.position.z

                            ans = np.dot(base_T_frame_matrix,
                                        frame_T_gripper_matrix)
                            ans2 = tft.quaternion_from_matrix(ans)
                            print("ans2: ", ans2)
                            goal_ps = PoseStamped()
                            goal_ps.pose.position.x = ans[0, 3]
                            goal_ps.pose.position.y = ans[1, 3]
                            goal_ps.pose.position.z = ans[2, 3]
                            goal_ps.pose.orientation.x = ans2[0]
                            goal_ps.pose.orientation.y = ans2[1]
                            goal_ps.pose.orientation.z = ans2[2]
                            goal_ps.pose.orientation.w = ans2[3]
                            goal_ps.header.frame_id = "base_link"
                            print("IN BASE_LINK:\n", ps_str(goal_ps))
                        else:
                            goal_ps = ps
                        print("Pose to move to: " + ps_str(goal_ps))
                        res = self.arm.move_to_pose(goal_ps)
                        # If moving was successful, break
                        if res is None:
                            print("Iteration", i, "failed.")
			    self.wiggle_head()
			    rospy.sleep(1.5)
                            if i == 2:
                                raise Exception("FAILED")
                        else:
                            print("Iteration", i, "successful.")
                            break

                    ########################################################
                    # Original
                    # if frame != "base_link":
                    #     listener.waitForTransform(
                    #         "base_link", frame, rospy.Time(), rospy.Duration(4.0))
                    #     while not rospy.is_shutdown():
                    #         try:
                    #             # now = rospy.Time.now()
                    #             now = rospy.Time(0)
                    #             listener.waitForTransform(
                    #                 "base_link", frame, now, rospy.Duration(4.0))
                    #             pos, rot = listener.lookupTransform(
                    #                 "base_link", frame, now)
                    #             break
                    #         except Exception as e:
                    #             print("Exception: " + str(e))
                    #             pass
                    #     base_T_frame_matrix = tft.quaternion_matrix(rot)
                    #     base_T_frame_matrix[0, 3] = pos[0]
                    #     base_T_frame_matrix[1, 3] = pos[1]
                    #     base_T_frame_matrix[2, 3] = pos[2]

                    #     ori = ps.pose.orientation
                    #     frame_T_gripper_matrix = tft.quaternion_matrix(
                    #         [ori.x, ori.y, ori.z, ori.w])
                    #     frame_T_gripper_matrix[0, 3] = ps.pose.position.x
                    #     frame_T_gripper_matrix[1, 3] = ps.pose.position.y
                    #     frame_T_gripper_matrix[2, 3] = ps.pose.position.z

                    #     ans = np.dot(base_T_frame_matrix,
                    #                  frame_T_gripper_matrix)
                    #     ans2 = tft.quaternion_from_matrix(ans)
                    #     print("ans2: ", ans2)
                    #     goal_ps = PoseStamped()
                    #     goal_ps.pose.position.x = ans[0, 3]
                    #     goal_ps.pose.position.y = ans[1, 3]
                    #     goal_ps.pose.position.z = ans[2, 3]
                    #     goal_ps.pose.orientation.x = ans2[0]
                    #     goal_ps.pose.orientation.y = ans2[1]
                    #     goal_ps.pose.orientation.z = ans2[2]
                    #     goal_ps.pose.orientation.w = ans2[3]
                    #     goal_ps.header.frame_id = "base_link"
                    #     print("IN BASE_LINK:\n", goal_ps)
                    # else:
                    #     goal_ps = ps
                    # res = arm.move_to_pose(goal_ps)
                    # if res is None:
                    #     print("Actually failed ------ T.T")
                    #     raise Exception
                elif command.type == Command.OPEN_GRIPPER:
                    self.gripper.open()
                elif command.type == Command.CLOSE_GRIPPER:
                    self.gripper.close()
                else:
                    print("UNKNOWN COMMAND " + str(command.type))
                rospy.sleep(1)
                print("Run succeeded!\n")
        except Exception as e:
            print("Run failed!")
            print(e)
