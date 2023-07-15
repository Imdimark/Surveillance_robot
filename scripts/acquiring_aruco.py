#!/usr/bin/env python
"""
Module Name:
    aruco_detection_service.py

Synopsis:
    This module defines a ROS service for detecting ArUco markers and 
    returning a list of the IDs of the detected markers. 

Author:
    Giovanni Di Marco <giovannidimarco06@gmail.com>

Date:
    15th July, 2023
"""
import rospy
from std_msgs.msg import Int32
from assignment2.srv import ArucoDetection, ArucoDetectionResponse
import threading

import moveit_commander
detected_ids = set()

def rotating(position):
    """
    Rotates the first joint of the group to a specified position.

    Args:
    position: The position in radians to rotate the joint to.
    """
    robot = moveit_commander.RobotCommander()
    group_name = "arm" # arm is the name of the joint group
    group = moveit_commander.MoveGroupCommander(group_name)

    # Obtaining actual joint position
    joint_goal = group.get_current_joint_values()

    # joint [0] is the first one, in this case rotational joint
    joint_goal[0] = position 

    # Moves joint in desidered position
    group.go(joint_goal, wait=True)

    # calling "stop" grants that there will not be residual movements
    group.stop()

def callback(data):
    """
    Callback for the /aruco_marker_publisher/detected_ids topic.

    Args:
    data: The message data.
    """
    global detected_ids
    # Insert detected ID into the set
    detected_ids.add(data.data)
    
def handle_aruco_detection(req):
    """
    Handler for the aruco_detection service. It rotates the robot's arm and 
    returns a list of the IDs of the detected ArUco markers.

    Args:
    req: The service request.

    Returns:
    An ArucoDetectionResponse with a list of the IDs of the detected markers.
    """
    rotating(3.14)
    rotating(-3.14)    
    print (list(detected_ids))
    return ArucoDetectionResponse(list(detected_ids))

def main():
    """
    Initializes the ROS node, subscriber, and service for detecting ArUco markers. 
    Spins the node to keep the service responsive.

    The node is named 'aruco_id_service' and the service is 'aruco_detection'. The node 
    subscribes to the '/aruco_marker_publisher/detected_ids' topic.

    When a message is received on the topic, the callback function is invoked to update 
    the detected_ids set with the ID from the message.

    When the service is called, it invokes the handle_aruco_detection function to rotate 
    the robot's arm, detect ArUco markers, and return a list of the IDs of the detected markers.
    """
    rospy.init_node('aruco_id_service', anonymous=True)
    rospy.Subscriber('/aruco_marker_publisher/detected_ids', Int32, callback)
    s = rospy.Service('aruco_detection', ArucoDetection, handle_aruco_detection)
    rospy.spin()

if __name__ == '__main__':
    main()

