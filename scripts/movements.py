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

def extracting_coordinates(target_room):

     # Get the path of the file
    assignment2_path = os.getenv('ASSIGNMENT2_PATH')
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

    return des_x, des_y

def movebase_client(client, des_x, des_y):
    
    rospy.loginfo("Waiting for move base server")
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'map' 
    goal.target_pose.pose.position.x = des_x
    goal.target_pose.pose.position.y = des_y
    goal.target_pose.pose.orientation.z = 0.727
    goal.target_pose.pose.orientation.w = 0.686
    client.send_goal(goal)

def simulating_movements(goal):
    armcli = ArmorClient("example", "ontoRef")
    target_room = goal.target_position 
    des_x, des_y = extracting_coordinates(target_room)    
    rospy.loginfo('Target room: ' + target_room)
    rospy.loginfo('coordinates: ' + str(des_x) + ', ' + str(des_y))
    success = False
    rospy.loginfo('Moving...')
    
    client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
    movebase_client(client,des_x, des_y)
    client.wait_for_result()

    state = client.get_state()
    if state == actionlib.GoalStatus.SUCCEEDED:
        rospy.loginfo("Goal raggiunto con successo")
        success = True
    elif state == actionlib.GoalStatus.PREEMPTED:
        rospy.loginfo("L'obiettivo è stato cancellato/preempted")
        success = False
    else:
        rospy.loginfo("Lo stato dell'azione è: %s" % state)
        input("Premi un tasto per continuare...")
    if success:
        armcli.call('REASON','','',[''])

        #updating robot position when the robot moves to a new location (target_room)
        actual_position = rospy.get_param('ActualPosition')
        armcli.call('REPLACE','OBJECTPROP','IND',['isIn', 'Robot1', target_room, actual_position]) 
        armcli.call('REASON','','',[''])
        
        #updating robot timestamp when the robot moves to a new location
        query_time = armcli.call('QUERY','DATAPROP','IND',['now', 'Robot1'])
        old_time = re.findall(r'\d+',query_time.queried_objects[0])[0] 
        actual_time = str(math.floor(time.time()))
        armcli.call('REPLACE','DATAPROP','IND',['now', 'Robot1', 'Long', actual_time, old_time])
        armcli.call('REASON','','',[''])

        #updating room timestamp "visitedat " when the robot visits a new location
        if "R" in target_room: #if is a room
            print ("target room is a room, im azzerando")
            query_time=armcli.call('QUERY','DATAPROP','IND',['visitedAt', target_room])
            old_time = re.findall(r'\d+',query_time.queried_objects[0])[0]
            armcli.call('REPLACE','DATAPROP','IND',['visitedAt', target_room, 'Long', actual_time, old_time])
            armcli.call('REASON','','',[''])
        
        
        print ("moved from" + actual_position + "to" + target_room)
        
        rospy.set_param('ActualPosition', target_room)
        rospy.loginfo("Move action succeeded, now robot is in room %s", target_room)
    
    return PlanningSrvResponse(success)
        
    

def movements_server():
    rospy.init_node("movements_server")
    s = rospy.Service('move_to_position', PlanningSrv, simulating_movements)
    rospy.loginfo("Ready to move robot.")
    rospy.spin()

if __name__ == "__main__":
    movements_server()