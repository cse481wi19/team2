import rospy
import pickle
import robot_api
import tf
import tf.transformations as tft
import numpy as np

from geometry_msgs.msg import PoseStamped
from command import Command

class Program(object):
    def __init__(self):
        self.commands = []

    def add_pose_command(self, ps, alias):
        self.commands.append(
            Command(Command.POSE, pose_stamped=ps, alias=alias))

    def add_open_gripper_command(self):
        self.commands.append(Command(Command.OPEN_GRIPPER))

    def add_close_gripper_command(self):
        self.commands.append(Command(Command.CLOSE_GRIPPER))

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

    def run(self, listener=None):
        if listener is None:
            listener = tf.TransformListener()
        try:
            arm = robot_api.Arm()
            gripper = robot_api.Gripper()
            rospy.sleep(1)
            for i, command in enumerate(self.commands):
                print(i, str(command))
                if command.type == Command.POSE:
                    print(command.pose_stamped)
                    ps = command.pose_stamped
                    curr_frame = command.pose_stamped.header.frame_id
                    if curr_frame != "base_link":
                        listener.waitForTransform(
                            "base_link", curr_frame, rospy.Time(), rospy.Duration(4.0))
                        while not rospy.is_shutdown():
                            try:
                                # now = rospy.Time.now()
                                now = rospy.Time(0)
                                listener.waitForTransform(
                                    "base_link", curr_frame, now, rospy.Duration(4.0))
                                (pos, rot) = listener.lookupTransform(
                                    "base_link", curr_frame, now)
                                break
                            except Exception as e:
                                print("Exception: " + str(e))
                                pass
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
                        ps = PoseStamped()
                        ps.pose.position.x = ans[0, 3]
                        ps.pose.position.y = ans[1, 3]
                        ps.pose.position.z = ans[2, 3]
                        ps.pose.orientation.x = ans2[0]
                        ps.pose.orientation.y = ans2[1]
                        ps.pose.orientation.z = ans2[2]
                        ps.pose.orientation.w = ans2[3]
                        ps.header.frame_id = "base_link"
                    res = arm.move_to_pose(ps)
                    if res is None:
                        print("Actually failed ------ T.T")
                        raise Exception
                elif command.type == Command.OPEN_GRIPPER:
                    gripper.open()
                elif command.type == Command.CLOSE_GRIPPER:
                    gripper.close()
                else:
                    print("UNKNOWN COMMAND " + str(command.type))
                rospy.sleep(1)
                print("Run succeeded!")
        except Exception as e:
            print("Run failed!")
            print(e)
