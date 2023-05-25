#!/usr/bin/python
import moveit_commander
import rospy
import roscpp

rospy.init_node('moving_robot')

robot = moveit_commander.RobotCommander()
group = moveit_commander.MoveGroupCommander("arm")

# Genera un target casuale
group.set_random_target()

plan = group.plan()
group.execute(plan)

