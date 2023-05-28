#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from assignment2.srv import ArucoDetection, ArucoDetectionResponse
import threading

detected_ids = set()
end_program = False


def callback(data):
    global detected_ids
    # Insert detected ID into the set
    detected_ids.add(data.data)
    

def check_timeout():
    global end_program
    rospy.sleep(30)  # sleep for 30 seconds
    end_program = True

def handle_aruco_detection(req):
    global end_program
    end_program = False
    
    timer_thread = threading.Thread(target=check_timeout)
    timer_thread.start()
    
    rate = rospy.Rate(5) # Set frequency for the loop (5Hz => every 0.2s)
    while not rospy.is_shutdown() and not end_program:
        rate.sleep()

    return ArucoDetectionResponse(list(detected_ids))

def main():
    rospy.init_node('aruco_id_service', anonymous=True)
    rospy.Subscriber('/aruco_marker_publisher/detected_ids', Int32, callback)
    s = rospy.Service('aruco_detection', ArucoDetection, handle_aruco_detection)
    rospy.spin()

if __name__ == '__main__':
    main()

