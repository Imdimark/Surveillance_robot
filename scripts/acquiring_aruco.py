#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from assignment2.srv import ArucoDetection, ArucoDetectionResponse
import threading

import moveit_commander
detected_ids = set()

def rotating(position):
    robot = moveit_commander.RobotCommander()
    group_name = "arm" # il nome del gruppo di giunti che controlli
    group = moveit_commander.MoveGroupCommander(group_name)

    # Ottieni la posizione corrente dei giunti
    joint_goal = group.get_current_joint_values()

    # Assume che tu stia cercando di muovere il primo giunto
    joint_goal[0] = position # Sostituisci 1.0 con l'angolo in radianti a cui vuoi muovere il giunto

    # Muovi il giunto alla posizione desiderata
    group.go(joint_goal, wait=True)

    # Chiamare "stop" garantisce che non ci sia movimento residuo
    group.stop()




def callback(data):
    global detected_ids
    # Insert detected ID into the set
    detected_ids.add(data.data)
    



def handle_aruco_detection(req):
    rotating(3.14)
    rotating(-3.14)    
    print (list(detected_ids))
    return ArucoDetectionResponse(list(detected_ids))

def main():
    rospy.init_node('aruco_id_service', anonymous=True)
    rospy.Subscriber('/aruco_marker_publisher/detected_ids', Int32, callback)
    s = rospy.Service('aruco_detection', ArucoDetection, handle_aruco_detection)
    rospy.spin()

if __name__ == '__main__':
    main()

