#!/usr/bin/env python

import rospy
import math
import moveit_commander
import tf
import sys
from geometry_msgs.msg import PoseStamped

def rotate_end_effector():
    # Initialize moveit_commander and rospy.
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('rotate_end_effector_node', anonymous=True)

    robot = moveit_commander.RobotCommander()

    # Get the specific group for the end effector
    group = moveit_commander.MoveGroupCommander("arm")

    # Get the current pose
    current_pose = group.get_current_pose().pose

    # Convert current orientation to euler
    euler = tf.transformations.euler_from_quaternion([
        current_pose.orientation.x,
        current_pose.orientation.y,
        current_pose.orientation.z,
        current_pose.orientation.w])

    # Add 180 degrees to the current orientation
    euler = [euler[0], euler[1], euler[2] + math.pi]

    # Convert back to quaternion
    quaternion = tf.transformations.quaternion_from_euler(*euler)

    # Update the target pose
    current_pose.orientation.x = quaternion[0]
    current_pose.orientation.y = quaternion[1]
    current_pose.orientation.z = quaternion[2]
    current_pose.orientation.w = quaternion[3]

    # Set the new pose as the target
    group.set_pose_target(current_pose)

    # Plan and execute
    group.go(wait=True)

    # Stop the group to ensure no residual movement
    group.stop()

if __name__ == '__main__':
    try:
        rotate_end_effector()
    except rospy.ROSInterruptException:
        pass

