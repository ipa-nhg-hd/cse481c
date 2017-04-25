#! /usr/bin/env python

from geometry_msgs.msg import Pose, Point, Quaternion
import fetch_api
import rospy
import sys


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


def main():
    rospy.init_node('cart_arm_demo')
    wait_for_time()
    argv = rospy.myargv()
    pose1 = Pose(
        Point(0.042, 0.384, 1.826), Quaternion(0.173, -0.693, -0.242, 0.657))
    pose2 = Pose(
        Point(0.047, 0.545, 1.822), Quaternion(-0.274, -0.701, 0.173, 0.635))
    gripper_poses = [pose1, pose2]

    torso = fetch_api.Torso()
    torso.set_height(fetch_api.Torso.MAX_HEIGHT)

    arm = fetch_api.Arm()

    def on_shutdown():
        arm.cancel_all_goals()
    rospy.on_shutdown(on_shutdown)

    while not rospy.is_shutdown():
        for pose in gripper_poses:
            error = arm.move_to_base_pose(pose)
            if error is not None:
                rospy.logerr(error)
            rospy.sleep(0.5)


if __name__ == '__main__':
    main()
