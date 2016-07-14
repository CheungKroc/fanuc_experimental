#!/usr/bin/env python

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import sys

from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Vector3Stamped
from geometry_msgs.msg import Wrench
from geometry_msgs.msg import WrenchStamped
import moveit_commander
from moveit_commander import conversions
import rospy
import tf
import tf2_geometry_msgs
import tf2_ros

import termcolor


def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('touch_floor')

    robot = moveit_commander.RobotCommander()
    arm = robot.manipulator
    scene = moveit_commander.PlanningSceneInterface()
    rospy.sleep(1)

    # THIS APPEARS
    from geometry_msgs.msg import PoseStamped
    p = PoseStamped()
    p.header.frame_id = robot.get_planning_frame()
    p.pose.orientation.w = 1
    scene.add_box('floor', p, (1, 1, 0.01))
    rospy.sleep(1)

    arm.clear_pose_targets()
    pose = arm.get_current_pose().pose
    pose.orientation = Quaternion(
        *tf.transformations.quaternion_from_euler(-3.14, 0, 0))
    arm.set_pose_target(pose)
    arm.go(wait=True)
    rospy.sleep(1)

    # THIS WON'T APPEAR
    from geometry_msgs.msg import PoseStamped
    p = PoseStamped()
    p.header.frame_id = robot.get_planning_frame()
    p.pose.position.z = -0.1
    p.pose.orientation.w = 1
    scene.add_box('floor2', p, (1, 1, 0.01))
    rospy.sleep(1)

    moveit_commander.roscpp_shutdown()
    moveit_commander.os._exit(0)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException():
        pass
