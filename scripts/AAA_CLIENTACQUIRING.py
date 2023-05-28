#!/usr/bin/env python
import rospy
from assignment2.srv import ArucoDetection

def call_service():
    rospy.wait_for_service('aruco_detection')

    try:
        aruco_detection = rospy.ServiceProxy('aruco_detection', ArucoDetection)
        response = aruco_detection()
        rospy.loginfo("Detected IDs: %s", response.ids)
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)

def main():
    rospy.init_node('aruco_detection_client_node')
    call_service()
    rospy.spin()

if __name__ == "__main__":
    main()

