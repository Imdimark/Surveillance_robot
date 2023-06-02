import rospy
import moveit_commander

rospy.init_node('moveit_example_node', anonymous=True)

robot = moveit_commander.RobotCommander()
group_name = "arm" # il nome del gruppo di giunti che controlli
group = moveit_commander.MoveGroupCommander(group_name)

# Ottieni la posizione corrente dei giunti
joint_goal = group.get_current_joint_values()

# Assume che tu stia cercando di muovere il primo giunto
joint_goal[0] = -3.14 # Sostituisci 1.0 con l'angolo in radianti a cui vuoi muovere il giunto

# Muovi il giunto alla posizione desiderata
group.go(joint_goal, wait=True)

# Chiamare "stop" garantisce che non ci sia movimento residuo
group.stop()

