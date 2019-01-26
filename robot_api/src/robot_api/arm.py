# TODO: import ?????????
# TODO: import ???????_msgs.msg
# TODO: import ??????????_msgs.msg
import rospy
import actionlib
from trajectory_msgs.msg import JointTrajectoryPoint
from trajectory_msgs.msg import JointTrajectory

from control_msgs.msg import JointTrajectoryAction
from control_msgs.msg import JointTrajectoryGoal

from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryGoal

from .arm_joints import ArmJoints


class Arm(object):
    """Arm controls the robot's arm.
    Joint space control:
        joints = ArmJoints()
        # Fill out joint states
        arm = robot_api.Arm()
        arm.move_to_joints(joints)
    """

    def __init__(self):
        # TODO: Create actionlib client
        self._client = actionlib.SimpleActionClient("arm_controller/follow_joint_trajectory", 
            FollowJointTrajectoryAction)
        # TODO: Wait for server
        self._client.wait_for_server()
        pass

    def move_to_joints(self, arm_joints):
        """Moves the robot's arm to the given joints.
        Args:
            arm_joints: An ArmJoints object that specifies the joint values for
                the arm.
        """
        # TODO: Create a trajectory point
        point = JointTrajectoryPoint()
        # TODO: Set position of trajectory point
        for value in arm_joints.values():
            point.positions.append(value)
            
        # point.positions.append(arm_joints.values())
        # TODO: Set time of trajectory point
        point.time_from_start = rospy.Duration(secs=5, nsecs=0)

        # TODO: Create goal
        goal = FollowJointTrajectoryGoal()

        traj = JointTrajectory()

        # TODO: Add joint name to list
        # traj.joint_names.append(ArmJoints.names())
        for value in ArmJoints.names():
            traj.joint_names.append(value)
        # TODO: Add the trajectory point created above to trajectory
        traj.points.append(point)
        goal.trajectory = traj

        # TODO: Send goal
        self._client.send_goal(goal)
        # TODO: Wait for result
        self._client.wait_for_result()
# rospy.logerr('Not implemented.')