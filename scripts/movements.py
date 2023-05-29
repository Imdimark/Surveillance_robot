import rospy
import actionlib
from time import sleep
from assignment1.msg import PlanningAction, PlanningResult, PlanningGoal
from armor_api.armor_client import ArmorClient
import math
import re
import time
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseFeedback, MoveBaseResult

def simulating_movements(goal):
    armcli = ArmorClient("example", "ontoRef")
    target_room = goal.target_room 
    print ("Target room" + target_room)
    success = True
    rospy.loginfo('Moving...')
    
    
    motion_duration = rospy.get_param('MovingDurationParam')
    if goal.skip_batterycancel == False:
        for counter in range(10*motion_duration):
            if server.is_preempt_requested():
                rospy.loginfo("Moving action preempted")
                success = False
                break
            
            sleep(0.1)
  
    else:
        sleep (motion_duration)


    result = PlanningResult()
    result.result = success
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
            query_time=armcli.call('QUERY','DATAPROP','IND',['visitedAt', target_room])
            old_time = re.findall(r'\d+',query_time.queried_objects[0])[0]
            armcli.call('REPLACE','DATAPROP','IND',['visitedAt', target_room, 'Long', actual_time, old_time])
            armcli.call('REASON','','',[''])
        
        
        print ("moved from" + actual_position + "to" + target_room)
        
        rospy.set_param('ActualPosition', target_room)
        server.set_succeeded(result)
        rospy.loginfo("Move action succeeded, now robot is in room %s", target_room)
    else:
        server.set_preempted(result)
    

if __name__ == "__main__":
    rospy.init_node("movements_server")
    
    server = actionlib.SimpleActionServer(
        "move_to_position",
        PlanningAction,
        simulating_movements,
        False
    )
    server.start()
    rospy.spin()
