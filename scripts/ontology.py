#!/usr/bin/env python
"""
Module Name:
    InitMapNode.py

Synopsis:
    This module defines a ROS node for initializing a map of the environment.

Author:
    Giovanni Di Marco <giovannidimarco06@gmail.com>

Date:
    15th July, 2023
"""

import rospy
import time
import rospkg
import subprocess
import re
import math
from std_msgs.msg import Bool
from std_srvs.srv import Empty
from armor_api.armor_client import ArmorClient
import rospy
from assignment2.srv import ArucoDetection
from assignment2.srv import RoomInformation 
import csv
import os 

room_coordinates = {}  
element_list = [] 

def request_room_info(marker_ids,armcli):
    """
    Requests room information for a given list of marker IDs.

    Args:
        marker_ids (list): A list of marker IDs for which to request room information.
        armcli (ArmorClient): An ArmorClient instance to interact with the knowledge base.

    Returns:
        list: A list of elements associated with the rooms.
    """    
    rospy.wait_for_service('/room_info')
    try:
        rospy.loginfo("Requesting room information:")
        room_info = rospy.ServiceProxy('/room_info', RoomInformation)
        for marker_id in marker_ids:
            response = room_info(marker_id)
            if response.room != "no room associated with this marker id":
                print("Room: ", response.room)
                print("Coordinates: ", response.x, response.y)
                room_coordinates[response.room] = (response.x, response.y)
                print("Connections:")
                element_list.append(response.room)
                for connection in response.connections:
                    armcli.call('ADD','OBJECTPROP','IND',['hasDoor', response.room, connection.through_door])
                    print("Connected to: ", connection.connected_to, "through door: ", connection.through_door)
                    element_list.append(connection.connected_to)
                    element_list.append(connection.through_door)
                # Save the room coordinates to a CSV file, to be used by other nodes    
                assignment2_path = os.getenv('ASSIGNMENT2_PATH')
                if assignment2_path is None:
                    rospy.logerr("Environment variable ASSIGNMENT2_PATH is not set!")
                    return
                file_path = os.path.join(assignment2_path, 'room_coordinates.csv')
                with open(file_path, 'w') as csv_file:
                    writer = csv.writer(csv_file)
                    # Write the headers
                    writer.writerow(["Room", "Coordinates"])
                    # Write the room coordinates
                    for key, value in room_coordinates.items():
                        writer.writerow([key, value])
        return element_list
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

def aruco_detection_client(armcli):
    """
    Calls the ArUco marker detection service and request room information for the detected markers.

    Args:
        armcli (ArmorClient): An ArmorClient instance to interact with the knowledge base.

    Returns:
        list: A list of elements associated with the rooms.
    """    
    rospy.wait_for_service('aruco_detection')
    try:
        aruco_detection = rospy.ServiceProxy('aruco_detection', ArucoDetection)
        response = aruco_detection()
        rospy.loginfo("Detected IDs: %s", response.ids)
        element_list =request_room_info(response.ids,armcli)
        return element_list
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)  
 
def service_callback(request):
    """
    Callback for the initmap_service. It initializes the map of the environment.

    Args:
        request (Empty): The request of the service call.

    Returns:
        list: An empty list.
    """
    rospy.wait_for_service('armor_interface_srv')
    print ("armor_interface loaded")

    # initialization of the map

    armcli = ArmorClient("example", "ontoRef")
    armcli.call('LOAD','FILE','',['/root/ros_ws/src/assignment2/topological_map/topological_map.owl', 'http://bnc/exp-rob-lab/2022-23', 'true', 'PELLET', 'false'])
    element_list = aruco_detection_client(armcli)
    element_list = list(set(element_list))
    print ("element list: ", element_list)
    
    for element in element_list:
        for element_nested in element_list:
            if element != element_nested:
                armcli.call('DISJOINT','IND','',[element, element_nested])
    
 
    armcli.call('REASON','','',[''])
    print ("starting adding visited at")
    print ("len element list: ", len(element_list))
    print ("type element list: ", type(element_list))


    for element in element_list:
        if "R" in element.upper():
            print ("element: ", element)
            print ("type element: ", type(element))
            armcli.manipulation.add_dataprop_to_ind("visitedAt", element, "Long", str(math.floor(time.time())))
    
    armcli.call('REASON','','',[''])
    
    armcli.call('ADD','OBJECTPROP','IND',['isIn', 'Robot1', 'E'])
    rospy.set_param('ActualPosition', 'E')
    armcli.call('REASON','','',[''])

    
    query_time = armcli.call('QUERY','DATAPROP','IND',['now', 'Robot1'])
    old_time = re.findall(r'\d+',query_time.queried_objects[0])[0] 
    actual_time = str(math.floor(time.time()))
    armcli.call('REPLACE','DATAPROP','IND',['now', 'Robot1', 'Long', actual_time, old_time])
    armcli.call('REASON','','',[''])

    
    rospy.loginfo('Map loaded, closing the node InitMapNode')
    
    return []

def main():
    """
    The main function to initialize the ROS node and the service.

    This function initiates a ROS node named 'InitMapNode', then it creates a service named 'initmap_service' 
    that uses the 'Empty' service type and is handled by 'service_callback' function.

    """
    rospy.init_node('InitMapNode')

    # Create the service 'initmap_service'
    service = rospy.Service('initmap_service', Empty, service_callback)
    
    rospy.spin()

if __name__ == '__main__':
    main()

