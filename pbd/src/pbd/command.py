class Command(object):
    POSE = 1
    OPEN_GRIPPER = 2
    CLOSE_GRIPPER = 3

    NUM_TO_CMD = {POSE: "Pose", OPEN_GRIPPER: "Open gripper",
                  CLOSE_GRIPPER: "Close gripper"}

    def ps_str(self, ps):
        pos = ps.pose.position
        rot = ps.pose.orientation
        return "PoseStamped(pos: (%.3f, %.3f, %.3f), rot: (%.3f, %.3f, %.3f, %.3f), frame_id: %s)" % (pos.x, pos.y, pos.z, rot.x, rot.y, rot.z, rot.w, ps.header.frame_id)


    def __init__(self, type, pose_stamped=None, alias=None):
        self.type = type
        self.pose_stamped = pose_stamped
        self.alias = alias

    def __str__(self):
        if self.type == self.POSE:
            return "Command(type=%s, alias='%s', frame_id='%s', ps=%s)" % (self.NUM_TO_CMD[self.type], str(self.alias), str(self.pose_stamped.header.frame_id), self.ps_str(self.pose_stamped))
        else:
            return "Command(type=%s)" % (self.NUM_TO_CMD[self.type])
