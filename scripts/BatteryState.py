#!/usr/bin/env python
"""
Module Name:
    batterystatus.py

Synopsis:
    This module provides a ROS node for managing the battery status of a robot.
    It checks the battery level at regular intervals and performs appropriate
    actions based on the current battery state.

Author:
    Giovanni Di Marco <giovannidimarco06@gmail.com>

Date:
    15th July, 2023

This is a ROS node for managing the battery status of a robot. It keeps track of the battery level and
controls the behaviour of the robot according to the battery state. When the battery is running too low or is empty,
it commands the robot to cancel its current tasks and head to the charging station.
"""
import rospy
import random
from std_msgs.msg import Bool
import roslaunch 
from std_srvs.srv import Empty
from assignment1.msg import PlanningAction,PlanningResult,PlanningGoal
from assignment2.srv import PlanningSrv, PlanningSrvResponse
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseFeedback, MoveBaseResult
import actionlib
batteryduration=720

def BatteryState():
    """
    Manages the state of the battery.

    This function keeps track of the battery level. If the battery is running low or is empty, 
    the robot will cancel its current tasks and go to the charging station. If the battery is 
    currently charging, it will monitor until it's fully charged. Status of the battery is pubblished throught the 
    topic BatteryState

    Args: 
        None

    Returns: 
        None

    """
    client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
    pub = rospy.Publisher('BatteryState', Bool, queue_size=10)
    rate = rospy.Rate(1) # 1 hz (charging frequency)
    batterylevel = batteryduration
    batteryBool = True
    CanCancelFlag = True # when the battery is empty, the robot can cancel the current goal but not the next one (going to the charging station)
    while not rospy.is_shutdown():
        ImCharging = rospy.get_param('IsChargingParam') # this parameter is true when the robot reaches "E"
        if (not ImCharging) and (batterylevel > 0): #discharging 
            batterylevel = batterylevel - 1
            if batterylevel < 7:
                rospy.loginfo("Battery is going too low")          
        elif (not ImCharging) and (batterylevel == 0): #Battery is empty
            batteryBool = False
            if CanCancelFlag: 
                client.cancel_all_goals()
                CanCancelFlag = False
                rospy.loginfo("Battery is empty, preempting current goal, going to charge station")
            rospy.loginfo("WARNING: Battery is empty")
        
        elif ImCharging and (batterylevel <= batteryduration): #charging 
            if batterylevel == batteryduration:
                rospy.loginfo("Battery is full")
                CanCancelFlag = True
                batteryBool = True
            else:
                batterylevel = batterylevel + 10 
                rospy.loginfo("Charging")
	
        rospy.loginfo("Battery level:" + str(batterylevel)) 
        pub.publish(batteryBool)
        rate.sleep()

if __name__ == '__main__':
    """
    The main execution block of the script.

    This block is executed when the script is run directly (not imported as a module). 
    It initializes the ROS node and calls the BatteryState function to start managing the battery state.

    It also handles the rospy.ROSInterruptException, which is raised when the node is manually shutdown.
    """
    try:
        rospy.init_node('batterystatus', anonymous=True)#, anonymous=True
        BatteryState()
    except rospy.ROSInterruptException:
        pass
