class Command(object):
    POSE = 1
    OPEN_GRIPPER = 2
    CLOSE_GRIPPER = 3

    NUM_TO_CMD = {POSE: "Pose", OPEN_GRIPPER: "Open gripper",
                  CLOSE_GRIPPER: "Close gripper"}

    def __init__(self, type, pose_stamped=None, alias=None):
        self.type = type
        self.pose_stamped = pose_stamped
        self.alias = alias

    def __str__(self):
        if self.type == self.POSE:
            return "Command(type=%s, alias='%s', frame_id='%s')" % (self.NUM_TO_CMD[self.type], str(self.alias), str(self.pose_stamped.header.frame_id))
        else:
            return "Command(type=%s)" % (self.NUM_TO_CMD[self.type])
