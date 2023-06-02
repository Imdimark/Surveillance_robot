import rospy
import actionlib
from time import sleep
#from assignment1.msg import PlanningAction, PlanningResult, PlanningGoal
from armor_api.armor_client import ArmorClient
import math
import re
import time
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseFeedback, MoveBaseResult
import csv
import os
from assignment2.srv import PlanningSrv, PlanningSrvResponse

rospy.init_node('sending_goal')


target_room = input ("inserisci stanza")
assignment2_path = "/root/ros_ws/src/assignment2"
if assignment2_path is None:
    print("Environment variable ASSIGNMENT2_PATH is not set!")
else:
    file_path = os.path.join(assignment2_path, 'room_coordinates.csv')

    # Initialize an empty dictionary to store the room coordinates
    room_coordinates = {}

    # Open the file and read the contents
    with open(file_path, 'r') as csv_file:
        reader = csv.reader(csv_file)
        next(reader)  # Skip the header
        for row in reader:
            # Convert the 'Coordinates' back to a tuple and store it in the dictionary
            room, coordinates = row[0], eval(row[1])
            room_coordinates[room] = coordinates


# Get the coordinates of the specific room
if target_room in room_coordinates:
    des_x = room_coordinates[target_room][0]
    des_y = room_coordinates[target_room][1]
else:
    print("Room not found.")





client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
rospy.loginfo("Waiting for move base server")
client.wait_for_server()

goal = MoveBaseGoal()
goal.target_pose.header.frame_id = 'map' 
goal.target_pose.pose.position.x = des_x
goal.target_pose.pose.position.y = des_y
goal.target_pose.pose.orientation.z = 0.727
goal.target_pose.pose.orientation.w = 0.686

client.send_goal(goal)
client.wait_for_result()
print (client.get_state())
print ("position reached")
