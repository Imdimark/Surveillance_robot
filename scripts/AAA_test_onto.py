#!/usr/bin/env python

import rospy
import time
import rospkg
import subprocess

from std_msgs.msg import Bool
#from assignment1 import Empty
from std_srvs.srv import Empty
from armor_api.armor_client import ArmorClient

def service_callback(request):
    '''url = 'https://github.com/buoncubi/topological_map/blob/main/topological_map.owl'
    filename = 'topological_map.owl'

    subprocess.call(['wget', url, '-O', filename])'''

    #rospy.wait_for_service('armor_interface_srv')

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
    armcli.call('ADD','OBJECTPROP','IND',['isIn', 'Robot1', 'E'])
    rospy.set_param('ActualPosition', 'E')
    #rospy.set_param('IsChargingParam', False)


    armcli.call('DISJOINT','IND','',['R1','R2','R3','R4','E','C1','C2','D1','D2','D3','D4','D5','D6','D7'])
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








