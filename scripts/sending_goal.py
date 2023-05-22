import actionlib
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseFeedback, MoveBaseResult

rospy.init_node('sending_goal')

client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
rospy.loginfo("Waiting for move base server")
client.wait_for_server()

goal = MoveBaseGoal()
goal.target_pose.header.frame_id = 'map' 
goal.target_pose.pose.position.x = +6.063
goal.target_pose.pose.position.y = -11.035
goal.target_pose.pose.orientation.z = 0.727
goal.target_pose.pose.orientation.w = 0.686

client.send_goal(goal)
client.wait_for_result()
print ("position reached")
