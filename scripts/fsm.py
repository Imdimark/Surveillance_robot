#!/usr/bin/env python
"""
Module Name:
    fsm_node.py

Synopsis:
    This module defines a ROS node that simulates a robot moving through various rooms and corridors. The robot has a battery which can run low, in which case it needs to charge. The robot can also reach urgent rooms and visit them. 

Author:
    Giovanni Di Marco <giovannidimarco06@gmail.com>

Date:
    15th July, 2023
"""

import random
import rospy
import smach
import smach_ros
import time
import moveit_commander
from std_msgs.msg import String, Bool
from armor_api.armor_client import ArmorClient
import actionlib
from actionlib import SimpleActionClient
from assignment1.msg import PlanningAction, PlanningResult, PlanningGoal
from std_srvs.srv import Empty
from assignment2.srv import PlanningSrv, PlanningSrvResponse
import math
import re

rotation = 3.14

def rotating(position, has_to_wait):
    """
    Rotates the robot to a given position.

    :param position: The target position for the robot.
    :type position: float
    :param has_to_wait: Determines whether the robot has to wait after reaching the target position.
    :type has_to_wait: bool
    """
    robot = moveit_commander.RobotCommander()
    group_name = "arm" # name of the joint group
    group = moveit_commander.MoveGroupCommander(group_name)

    # Obteins the actual joint state
    joint_goal = group.get_current_joint_values()

    # Settin joint 0 the one to move
    joint_goal[0] = position #

    # moves joint to desidered position
    group.go(joint_goal, wait=has_to_wait)

    # Calling "stop" grants no residual moviments
    group.stop()
       
def extract_values(strings_list):
    """
    Extracts the values from a list of strings.

    :param strings_list: The list of strings from which values are to be extracted.
    :type strings_list: list
    :returns: A list of the extracted values.
    :rtype: list
    """
    matched_substrings = []
    for string in strings_list:
        start_index = string.find("#")
        end_index = string.find(">")
        while start_index != -1 and end_index != -1:
            substring = string[start_index + 1 : end_index]
            matched_substrings.append(substring)
            start_index = string.find("#", end_index)
            end_index = string.find(">", start_index)
    return matched_substrings

def choose_randomly(strings_list, character):
    """
    Selects a random string from a list of strings.

    :param strings_list: The list of strings from which a string is to be randomly selected.
    :type strings_list: list
    :param character: The character to be searched in the string.
    :type character: str
    :returns: The selected string.
    :rtype: str
    """
    selected_string = None
    actual_position = rospy.get_param('ActualPosition')
    """if actual_position in strings_list: ##### already implemented in the ontology
        strings_list.remove(actual_position)"""
    while selected_string is None or character not in selected_string:
        selected_string = random.choice(strings_list)
        
    return selected_string

def move_to_position_client(client, x, skip):
    """
    Calls a service to move the robot to a given position.

    :param client: The ROS service client.
    :type client: rospy.ServiceProxy
    :param x: The target position.
    :type x: float
    :param skip: Determines whether to skip the movement or not.
    :type skip: bool
    :returns: The response from the service call.
    :rtype: PlanningSrvResponse
    """
    resp = client(x)
    print ("result:", resp)
    return resp


class WaitForMapState(smach.State):
    """
    Defines the state where the robot waits for the map to be loaded.
    """
    def __init__(self):
    
        smach.State.__init__(self, outcomes=['map_loaded'])
        self.service_client = rospy.ServiceProxy('initmap_service', Empty)

    def execute(self, userdata):
        rospy.loginfo('Waiting for map to be loaded...')
        rospy.wait_for_service('initmap_service')
        
        response = self.service_client()
        
        return 'map_loaded'

class MoveInCorridorsState(smach.State):
    """
    Defines the state where the robot moves in the corridors.
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['battery_low', 'urgent_room_reached', 'no_urgent_available'], output_keys=['MoveInCorridorsState_output'])
        self.armcli = ArmorClient("example", "ontoRef")
        rospy.wait_for_service('move_to_position')
        self.client = rospy.ServiceProxy('move_to_position', PlanningSrv)
   
    def execute(self, userdata):      
        reachable_place_list_and_urgent = []
        #self.armcli.call('REASON','','',[''])
        canreach = self.armcli.call('QUERY','OBJECTPROP','IND',['canReach', 'Robot1'])
        

        reachable_place_list = extract_values (canreach.queried_objects)
        print ("can reach:", reachable_place_list)
        
        new__target_position = choose_randomly (reachable_place_list, "C") #C are all the corridors available <---------------------------
        result = move_to_position_client(self.client, new__target_position, False)
        print ("result:", result)
        
        isurgent_list_query =  self.armcli.call('QUERY','IND','CLASS',['URGENT'])
        isurgent_list = extract_values (isurgent_list_query.queried_objects)
        
        ############################################################################        
        canreach = self.armcli.call('QUERY','OBJECTPROP','IND',['canReach', 'Robot1'])
        reachable_place_list = extract_values (canreach.queried_objects)       
        reachable_place_list_and_urgent = list(set(reachable_place_list).intersection(isurgent_list))
        ###########################################################################

        if result.success:
            rospy.loginfo("Goal position reached")
            if reachable_place_list_and_urgent == []:
                rospy.loginfo("Goal position reached, no urgent rooms, continuing moving in corridors...")
                return 'no_urgent_available'
            else:
                rospy.loginfo("Goal position reached, urgent room found, moving to room...") ##the randomness will choose in the choose_randomly function
                userdata.MoveInCorridorsState_output = reachable_place_list_and_urgent ## giving to the next state the list of urgent and reachable rooms 
                return 'urgent_room_reached'
                
        else:
            rospy.loginfo("Goal was preempted or failed")
            return 'battery_low'
        

class VisitRoomState(smach.State):
    """
    Defines the state where the robot visits a room.
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['room_visited', 'battery_low'], input_keys=['MoveInCorridorsState_output'])
        self.armcli = ArmorClient("example", "ontoRef")
        self.batterystate = rospy.Subscriber('BatteryState', Bool, self.battery_callback)
        rospy.wait_for_service('move_to_position')
        self.client = rospy.ServiceProxy('move_to_position', PlanningSrv)
    
    def battery_callback(self, msg):
        self.bs = msg.data
    
    def execute(self, userdata):
        global rotation
        rospy.loginfo('Visiting room...')     
        
        new__target_position = choose_randomly (userdata.MoveInCorridorsState_output, "R") #R are all the reachable room available <---------------
        
        result = move_to_position_client(self.client, new__target_position, False)

        if result.success:
            rospy.loginfo("Goal position reached, im in room")
            
        else:
            rospy.loginfo("Goal was preempted,going to charging station state")
            return 'battery_low'
        
        start_time = rospy.Time.now()  # ottieni il tempo corrente
        rate = rospy.Rate(10)  # imposta la frequenza di esecuzione del loop a 10 Hz
        
        
        rospy.loginfo('Ispetioning the room... ')
        rotating (rotation, False) ##do not wait, if the battery is empty this avoid the malfunction
        
        
        rotation = -rotation
        while (rospy.Time.now() - start_time).to_sec() < rospy.get_param('RoomInspectionTime'):
            if not self.bs:
                return 'battery_low'
            rate.sleep()  # attendi il tempo necessario per mantenere la frequenza impostata

        
        rospy.loginfo('Room inspected, going back to corridor...')
        return 'room_visited'

class ChargingState(smach.State):
    """
    Defines the state where the robot charges its battery.
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['battery_full'])
        self.armcli = ArmorClient("example", "ontoRef")
        rospy.wait_for_service('move_to_position')
        self.client = rospy.ServiceProxy('move_to_position', PlanningSrv)
        
        
        self.batterystate = rospy.Subscriber('BatteryState', Bool, self.battery_callback)
    
    def battery_callback(self, msg):
        self.bs = msg.data

    def execute(self, userdata):
        global rotation
        rospy.loginfo('Moving to charging station...')
        
        #is different respect the assignment1 because it can find the charging station in the corridor directly (saving time)
        """canreach = self.armcli.call('QUERY','OBJECTPROP','IND',['canReach', 'Robot1'])
        reachable_place_list = extract_values (canreach.queried_objects)
        new__target_position = choose_randomly (reachable_place_list, "C") #C are all the corridors available

        if "E" in reachable_place_list:
            rospy.loginfo('Moving to charging station...')
            new__target_position = "E"
            result = move_to_position_client(self.client, new__target_position,True)

        else:
            rospy.loginfo('Moving to corridor before going to charging station...')                   
            new__target_position = choose_randomly (reachable_place_list, "C") #C are all the corridors available
            result = move_to_position_client(self.client, new__target_position,True)
            rospy.loginfo('Moving to charging station...')
            new__target_position = "E"
            result = move_to_position_client(self.client, new__target_position,True)"""

        new__target_position = "E"
        result = move_to_position_client(self.client, new__target_position,True)


        rospy.loginfo('Charging...')
        rospy.set_param('/IsChargingParam', True)
        
        
        while (self.bs == False):{
            time.sleep(1)
        }
        rotation = -3.14
        rotating (rotation, False) ##resetting during the movement
        rotation = 3.14
        rospy.loginfo('...Charged')
        rospy.set_param('/IsChargingParam', False)
        return 'battery_full'

def main():
    """
    Initializes the ROS node, creates a state machine for the robot, and starts an introspection server to visualize the state machine.
    """
    rospy.init_node('fsm_node')

    # Create the top-level SMACH state machine
    sm = smach.StateMachine(outcomes=['mission_completed'])

    # Open the container
    with sm:
        # Add states to the SMACH state machine
        smach.StateMachine.add('WAIT_FOR_MAP', WaitForMapState(),
                               transitions={'map_loaded': 'MOVE_IN_CORRIDORS'})
        smach.StateMachine.add('MOVE_IN_CORRIDORS', MoveInCorridorsState(),
                               transitions={'battery_low': 'CHARGING',
                                            'urgent_room_reached': 'VISIT_ROOM',
                                            'no_urgent_available': 'MOVE_IN_CORRIDORS'})
        smach.StateMachine.add('VISIT_ROOM', VisitRoomState(),
                               transitions={'room_visited': 'MOVE_IN_CORRIDORS',
                                            'battery_low': 'CHARGING'})
        smach.StateMachine.add('CHARGING', ChargingState(),
                               transitions={'battery_full': 'MOVE_IN_CORRIDORS'})

    # Create and start the introspection server to visualize the SMACH state machine
    
    sis = smach_ros.IntrospectionServer('fsm', sm, '/SM_ROOT')
    sis.start()

    # Execute the SMACH state machine
    outcome = sm.execute()

    # Stop the introspection server
    sis.stop()

    # Print the outcome of the SMACH state machine
    rospy.loginfo('Mission completed with outcome: %s' % outcome)

if __name__ == '__main__':
    main()
