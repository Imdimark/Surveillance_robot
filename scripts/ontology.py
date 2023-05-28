#!/usr/bin/env python

import rospy
import time
import rospkg
import subprocess
import re
import math
from std_msgs.msg import Bool
#from assignment1 import Empty
from std_srvs.srv import Empty
from armor_api.armor_client import ArmorClient
import rospy
from assignment2.srv import ArucoDetection


def aruco_detection_client():
    #starting rotating   <-------------- 
    

    rospy.wait_for_service('aruco_detection')

    try:
        aruco_detection = rospy.ServiceProxy('aruco_detection', ArucoDetection)
        response = aruco_detection()
        rospy.loginfo("Detected IDs: %s", response.ids)
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)




def service_callback(request):

    rospy.wait_for_service('armor_interface_srv')
    
    aruco_detection_client()
    
    
    
    # initialization of the map

    armcli = ArmorClient("example", "ontoRef")
    armcli.call('LOAD','FILE','',['/root/ros_ws/src/assignment1/topological_map/topological_map.owl', 'http://bnc/exp-rob-lab/2022-23', 'true', 'PELLET', 'false'])
    armcli.call('ADD','OBJECTPROP','IND',['hasDoor', 'E', 'D6'])
    armcli.call('ADD','OBJECTPROP','IND',['hasDoor', 'E', 'D7'])
    armcli.call('ADD','OBJECTPROP','IND',['hasDoor', 'R1', 'D1'])
    armcli.call('ADD','OBJECTPROP','IND',['hasDoor', 'R2', 'D2'])
    armcli.call('ADD','OBJECTPROP','IND',['hasDoor', 'R3', 'D3'])
    armcli.call('ADD','OBJECTPROP','IND',['hasDoor', 'R4', 'D4'])
    armcli.call('ADD','OBJECTPROP','IND',['hasDoor', 'C1', 'D1'])
    armcli.call('ADD','OBJECTPROP','IND',['hasDoor', 'C1', 'D2'])
    armcli.call('ADD','OBJECTPROP','IND',['hasDoor', 'C1', 'D5'])
    armcli.call('ADD','OBJECTPROP','IND',['hasDoor', 'C1', 'D6'])
    armcli.call('ADD','OBJECTPROP','IND',['hasDoor', 'C2', 'D3'])
    armcli.call('ADD','OBJECTPROP','IND',['hasDoor', 'C2', 'D4'])
    armcli.call('ADD','OBJECTPROP','IND',['hasDoor', 'C2', 'D5'])
    armcli.call('ADD','OBJECTPROP','IND',['hasDoor', 'C2', 'D7'])
    armcli.call('DISJOINT','IND','',['R1','R2','R3','R4','E','C1','C2','D1','D2','D3','D4','D5','D6','D7'])
    armcli.call('REASON','','',[''])

      
    
    armcli.manipulation.add_dataprop_to_ind("visitedAt", "R1", "Long", str(math.floor(time.time())))
    #rospy.sleep(random.uniform(minwait, maxwait))
    
    armcli.manipulation.add_dataprop_to_ind("visitedAt", "R2", "Long", str(math.floor(time.time())))
    #rospy.sleep(random.uniform(minwait, maxwait))
    
    armcli.manipulation.add_dataprop_to_ind("visitedAt", "R3", "Long", str(math.floor(time.time())))
    #rospy.sleep(random.uniform(minwait, maxwait))
    
    armcli.manipulation.add_dataprop_to_ind("visitedAt", "R4", "Long", str(math.floor(time.time())))
    #rospy.sleep(random.uniform(minwait, maxwait))
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
    rospy.init_node('InitMapNode')

    # Create the service
    service = rospy.Service('initmap_service', Empty, service_callback)
    
    rospy.spin()

if __name__ == '__main__':
    main()







