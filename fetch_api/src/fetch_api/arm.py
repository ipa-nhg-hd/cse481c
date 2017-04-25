#!/usr/bin/env python

import actionlib
import control_msgs.msg
import geometry_msgs.msg
import trajectory_msgs.msg
import math
import rospy

from .arm_joints import ArmJoints
from moveit_msgs.msg import MoveItErrorCodes
from moveit_python import MoveGroupInterface, PlanningSceneInterface

ARM_GROUP_NAME = 'arm'
ACTION_SERVER = 'arm_controller/follow_joint_trajectory'
TIME_FROM_START = 5


def moveit_error_string(val):
    """Returns a string associated with a MoveItErrorCode.

    Args:
        val: The val field from moveit_msgs/MoveItErrorCodes.msg

    Returns: The string associated with the error value, 'UNKNOWN_ERROR_CODE'
        if the value is invalid.
    """
    if val == MoveItErrorCodes.SUCCESS:
        return 'SUCCESS'
    elif val == MoveItErrorCodes.FAILURE:
        return 'FAILURE'
    elif val == MoveItErrorCodes.PLANNING_FAILED:
        return 'PLANNING_FAILED'
    elif val == MoveItErrorCodes.INVALID_MOTION_PLAN:
        return 'INVALID_MOTION_PLAN'
    elif val == MoveItErrorCodes.MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE:
        return 'MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE'
    elif val == MoveItErrorCodes.CONTROL_FAILED:
        return 'CONTROL_FAILED'
    elif val == MoveItErrorCodes.UNABLE_TO_AQUIRE_SENSOR_DATA:
        return 'UNABLE_TO_AQUIRE_SENSOR_DATA'
    elif val == MoveItErrorCodes.TIMED_OUT:
        return 'TIMED_OUT'
    elif val == MoveItErrorCodes.PREEMPTED:
        return 'PREEMPTED'
    elif val == MoveItErrorCodes.START_STATE_IN_COLLISION:
        return 'START_STATE_IN_COLLISION'
    elif val == MoveItErrorCodes.START_STATE_VIOLATES_PATH_CONSTRAINTS:
        return 'START_STATE_VIOLATES_PATH_CONSTRAINTS'
    elif val == MoveItErrorCodes.GOAL_IN_COLLISION:
        return 'GOAL_IN_COLLISION'
    elif val == MoveItErrorCodes.GOAL_VIOLATES_PATH_CONSTRAINTS:
        return 'GOAL_VIOLATES_PATH_CONSTRAINTS'
    elif val == MoveItErrorCodes.GOAL_CONSTRAINTS_VIOLATED:
        return 'GOAL_CONSTRAINTS_VIOLATED'
    elif val == MoveItErrorCodes.INVALID_GROUP_NAME:
        return 'INVALID_GROUP_NAME'
    elif val == MoveItErrorCodes.INVALID_GOAL_CONSTRAINTS:
        return 'INVALID_GOAL_CONSTRAINTS'
    elif val == MoveItErrorCodes.INVALID_ROBOT_STATE:
        return 'INVALID_ROBOT_STATE'
    elif val == MoveItErrorCodes.INVALID_LINK_NAME:
        return 'INVALID_LINK_NAME'
    elif val == MoveItErrorCodes.INVALID_OBJECT_NAME:
        return 'INVALID_OBJECT_NAME'
    elif val == MoveItErrorCodes.FRAME_TRANSFORM_FAILURE:
        return 'FRAME_TRANSFORM_FAILURE'
    elif val == MoveItErrorCodes.COLLISION_CHECKING_UNAVAILABLE:
        return 'COLLISION_CHECKING_UNAVAILABLE'
    elif val == MoveItErrorCodes.ROBOT_STATE_STALE:
        return 'ROBOT_STATE_STALE'
    elif val == MoveItErrorCodes.SENSOR_INFO_STALE:
        return 'SENSOR_INFO_STALE'
    elif val == MoveItErrorCodes.NO_IK_SOLUTION:
        return 'NO_IK_SOLUTION'
    else:
        return 'UNKNOWN_ERROR_CODE'


class Arm(object):
    """Arm controls the robot's arm.

    Joint space control:
        joints = ArmJoints()
        # Fill out joint states
        arm = fetch_api.Arm()
        arm.move_to_joints(joints)
    """
    BASE_FRAME = 'base_link'
    EE_FRAME = 'wrist_roll_link'

    def __init__(self):
        self._client = actionlib.SimpleActionClient(
            ACTION_SERVER, control_msgs.msg.FollowJointTrajectoryAction)
        self._client.wait_for_server(rospy.Duration(10))
        self._move_group = MoveGroupInterface(ARM_GROUP_NAME, Arm.BASE_FRAME)

    def move_to_joints(self, joint_state):
        goal = control_msgs.msg.FollowJointTrajectoryGoal()
        goal.trajectory.joint_names.extend(ArmJoints.names())
        point = trajectory_msgs.msg.JointTrajectoryPoint()
        point.positions.extend(joint_state.values())
        point.time_from_start = rospy.Duration(TIME_FROM_START)
        goal.trajectory.points.append(point)
        self._client.send_goal(goal)
        self._client.wait_for_result(rospy.Duration(10))

    def move_to_base_pose(self, pose):
        """Moves the end-effector to a pose, using motion planning.

        Args:
            pose: geometry_msgs/Pose, the goal pose for the gripper in the
                Arm.BASE_FRAME frame.

        Returns:
            string describing the error if an error occurred, else None.
        """
        ps = geometry_msgs.msg.PoseStamped()
        ps.header.frame_id = Arm.BASE_FRAME
        ps.header.stamp = rospy.Time.now()
        ps.pose = pose
        self._move_group.moveToPose(ps, Arm.EE_FRAME, wait=False)
        move_action = self._move_group.get_move_action()
        move_action.wait_for_result(rospy.Duration(20))
        result = move_action.get_result()
        if result:
            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                return None
            else:
                return moveit_error_string(result.error_code.val)
        else:
            rospy.logerr(
                'MoveIt did not return an error code, returning FAILURE')
            return moveit_error_string(MoveItErrorCodes.FAILURE)

    def cancel_all_goals(self):
        self._move_group.get_move_action().cancel_all_goals()
        self._client.cancel_all_goals()
