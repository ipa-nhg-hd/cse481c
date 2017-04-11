#!/usr/bin/env python

import math
import rospy

from .arm_joints import ArmJoints


class Arm(object):
    """Arm controls the robot's arm.

    Joint space control:
        joint_state = JointState()
        # Fill out joint states
        arm = fetch_api.Arm()
        arm.move_to_joints(joint_state)
    """

    def __init__(self):
        pass

    def move_to_joints(self, joint_state):
        rospy.logerr('Not implemented.')
