# Experimental robotics laboratory - Assignment2

## Index of contents:
1. [Introduction](#introduction)
2. [Behaviour](#video)
3. [Software architecture](#soft)
4. [Installation and running procedure](#installation)
5. [Working hypothesis and environment](#hyp)
6. [Authors and contacts](#contacts)




## Introduction <a name="introduction"></a>
The project at hand focuses on developing and implementing a surveillance robot, engineered to operate within an indoor environment. The environment consists of a 3D space with four rooms and three corridors. The robot moves using two weeks in space and uses a rotating arm for scanning the environment. For an initial starting point, the robot is placed in a specific location referred to as 'E'  (x = -6.0, y = 11.0).
It is an extension of the project carried on in the first assignment, implementing some autonomous navigation features and some computer vision for building ontology (scanning ARUCO markers).

<img src="https://github.com/Imdimark/SmachRobot_ROS/assets/78663960/471cb60b-42c2-490f-9482-4c0e266a9d8f" width="40%" height="40%">

The development of this package primarily leverages two other packages: 

  1. [SMACH](http://wiki.ros.org/smach) for implementing and managing the finite state machine
  2. [ARMOR](http://wiki.ros.org/smach) in order to load, query and modify multiple ontologies and requires very little knowledge of OWL APIs and Java
  3. [vision_opencv](https://github.com/ros-perception/vision_opencv) for implementing cv tasks
  4. [ARUCO](https://github.com/Imdimark/Moveit-package-for-surveillance-robot) for implementing Aruco identification
  5. [Gmapping](http://wiki.ros.org/gmapping) for implementing laser-based SLAM
  6. [Move_base](http://wiki.ros.org/move_base) links together a global and local planner to accomplish its global navigation task
  7. [MoveIt](https://moveit.ros.org/) for controlling the ik of the manipulator
  8. [assignment2](https://github.com/CarmineD8/assignment2/) package containing marker_server and the Gazebo's environment

## Behaviour <a name="video"></a>
 Robot start his behavior by scanning the Aruco markers placed in room "E" through the rotation of the arm. The number scanned are converted by the node [marker_server](https://github.com/CarmineD8/assignment2/blob/main/src/marker_server.cpp) that implements the server that converts numbers into information regarding the room. The operational structure of the robot is based on a topological map, which it constructs using incoming data about the connections between various rooms, corridors, and doorways. This information aids the robot in maneuvering around its environment and effectively fulfilling its surveillance duties.
The robot's actions follow an endless cycle: it moves to a new location, waits for a while, and then moves again. This process continues until the robot's energy levels run low, at which point it returns to location 'E' for a recharge before resuming its routine.
When the robot's battery is not low, it navigates through the environment with a specific policy in mind. It's programmed to stay primarily in the corridors, keeping a closer eye on them. However, if a nearby room hasn't been inspected for a while, the robot will deviate from its path to check the room, ensuring complete and thorough surveillance. 
The robot is designed to primarily operate within the corridors, dedicating 80% of its operational time, or 720 seconds, to moving and performing functions in these areas. This aligns with its nature as a monitoring or surveillance device.
When not in the corridors, the robot spends the remaining 20% of its time, or 180 seconds, visiting and checking the rooms. This visit includes moving from the corridor area to the room, checking the room's conditions, and returning to the corridor.
The robot is equipped with a battery with an autonomy of 900 seconds (15 minutes). Once the battery is completely drained, the robot autonomously returns to the charging station. This journey does not entail any energy consumption. After the battery has been fully charged, the robot resumes its activities by returning to the corridors or room areas as scheduled.
To ensure that all rooms are regularly visited and checked, the robot operates with an urgency threshold of 2160 seconds (36 minutes). This means that each room is visited and checked at least once within this time interval.
Moreover, the robot is programmed to return to the point where it interrupted its path after recharging, allowing it to resume its activities without interruptions.
This behavior ensures that the robot can perform its activities efficiently, adhering to energy limitations and operational priorities, while ensuring a regular check of all rooms.

https://github.com/Imdimark/assignment2/assets/78663960/b07a4abc-9370-47d6-8ba6-0cf3c8a66005


This video shows how the state machine works and goes through all the states and how the simulation of the robots moves autonomously. There are two simulation windows for Rviz and Gazebo. The other four windows, made by gnome terminals represent the four nodes that are running:

- **state of the battery** that is going up or down depending if it is discharging or charging (different state machine states)
- **movements** that shows the state of the movements like where the robot is, where the robot chan goes, and if the movements have been done
- **ontology** this node is in charge to contact the ARMOR service for the ontology and initialize the map and the visited time of the rooms (useful for the urgency threshold), the windows show up when the process is done and the fsm can proceed to the next state.
- **FSM** is the core of the system, this window shows up the changing of the states and some info. To better understand is suggested to follow state changes throughout the smach_viewer. 

## Software architecture <a name="sofar"></a>

### software components:
#### InitMapNode node
This node provides a ROS service named 'initmap_service', which is used to initialize a topological map used for robot navigation. This node receives from the node aruco_id_service the list of the marker id and sends them to the node aruco_id_service that extracts information.
The external communications of this component are mainly handled through ROS:
- initmap_service is a service called by the fsm node at the beginning of the finite state machine.
- ActualPosition parameter used to set the knowledge of the actual position, this parameter is used by the movements node to have the knowledge about the starting position and update the onology with the appropriate query. In this specific node, it initializes the starting position in corridor "E".
- armor_interface_srv is the service, client-side, that interacts with the armor server for  load, query, and modify the ontology.
- ArucoDetection service, client-side, that scans the aruco markers and returns a list containing markers id.
- RoomInformation service, client-side, that send the previous list and obtains a .csv containing information about the environment's configuration. In this file every is is associated univocally with one 2D coordinate.
#### aruco_id_service node
Defines a ROS node and service for detecting ArUco markers and returning their IDs. 
External communications:
- /aruco_marker_publisher/detected_ids topic, which publishes detected ArUco marker IDs. The callback function is invoked every time an Int32 message is received on this topic, adding the detected ID to the detected_ids set.
- aruco_detection server-side implementation through the handle_aruco_detection function which rotates the robot's arm and returns a list of the detected ArUco marker IDs.
- moveit_commander  is an interface to the MoveIt stands in this [package](https://github.com/Imdimark/Moveit-package-for-surveillance-robot).
  
#### batterystatus node
The node 'batterystatus' manages the battery status of a robot. 
External communications: 
- move_base SimpleActionClient
- BatteryState topic published by this node to monitor the battery status by other nodes, 
- IsChargingParam ROS parameter, this node gets this parameter at every cycle iteration to understand if the robot is in corridor "E" at the charging station. This parameter is useful to see if the robot is currently charging.
 - move_to_position is a simpleactionclient used to cancel the moving (simulated by wasting time) when the battery is empty, to be able to receive the new one of moving in the charging station.

#### movements_server node
It is designed to implement the movements, autonomously, of a robot. It retrieves the position from a .csv file in which every id (from the [InitMapNode node](https://github.com/Imdimark/EXPROLAB_Assignment2/blob/main/scripts/ontology.py) ) is univocally associated with a 2D coordinate used for reaching by the autonomous robot the position (specific room or corridor)
External communications:
- move_to_position server-side service, the server listens on the 'move_to_position' action for PlanningAction goals. The goal message includes the target room and a flag indicating whether to skip battery checks. 
- move_base action server: The node sends goals to this server to move the robot.
- move_to_position server side, allows other nodes to request the robot to move to a specific position. The service type is PlanningSrv, and the service callback is sending_goal_movements(goal).
- ActualPosition parameter to track and update the robot's current position.
- armor_interface_srv is the service, client-side, that interacts with the armor server for  load, query, and modify the ontology

#### fsm_node node
The FSM consists of four states, each represented by a smach.State class. This program makes the robot move and behave according to its battery level and the rooms it should visit.
External communications:
- BatteryState topic subscribed to monitor the robot's battery level
- initmap_service service, clientside, to call the creation of the ontology
- move_to_position SimpleActionClient which is likely responsible for controlling the robot's movements through the PlanningAction action.
- RoomInspectionTime e IsChargingParam are ros parameter that respectevely set
 for how much time the robot is inspecting the room (time for compleating 1 rotation) useful for checking the battery state and the second one to set when the robot is in the charging state.

### State Viewpoint
The following schema represents the possible states and when transition could happen 

<img src="https://github.com/Imdimark/SmachRobot_ROS/assets/78663960/d8306a3a-8e4d-4c79-a1b3-f12376af0b95" width="60%" height="60%">

The states are WAIT_FOR_MAP, MOVE_IN_CORRIDORS, VISIT_ROOM, and CHARGING.

- WAIT_FOR_MAP in this state, the robot waits for a map to be loaded, which it uses for navigation.
- MOVE_IN_CORRIDORS  in this state, the robot moves in the corridors. It continues to move until its battery is low or an urgent room needs to be visited (in this case the state change only at the end of the movement.
- VISIT_ROOM  in this state, the robot visits a room. If the battery is low, it transitions to the charging state. Once the room has been visited, the robot goes back to moving in the corridors.
- CHARGING in this state, the robot moves to the charging station and starts charging its battery. Once the battery is fully charged, the robot goes back to moving in the corridors.
### Nodes 
The following schema is a rqt_graph generated starting from the running ros nodes and represents their architecture and how the nodes communicate with each other.

<img src="https://github.com/Imdimark/assignment2/assets/78663960/6fb8f6b4-4baf-43d9-9ac5-132abbc27285" width="60%" height="60%">
### Smach state machine
As we said smach is a fundamental component for the entire architecture, leading the implementation of the finite state machine. As we can see in the video, through the command: ```rosrun smach_viewer smach_viewer.py``` we can follow actual state changes.
<img src="https://github.com/Imdimark/SmachRobot_ROS/assets/78663960/7cb4dc51-8ce3-4f5f-a7c0-62932a981d32" width="60%" height="60%">

## Installation and running procedure <a name="installation"></a>
Some generic requirements can be necessary(like Python and ros), but the suggestion is to start with this container (based on Linux):  [carms84/exproblab](https://hub.docker.com/r/carms84/exproblab) 

Some mandatory prerequisites are needed:
- ```git clone https://github.com/Imdimark/Moveit-package-for-surveillance-robot``` inside the workspace
- ```https://github.com/Imdimark/aruco_ros``` inside the workspace
- Download the project in your workspace:
  - ```cd <myworkspace>/src/```
  - ```git clone https://github.com/Imdimark/EXPROLAB_Assignment2```
- Install gnome terminal: ```sudo apt-get install gnome-terminal```
- Install armor, following this guideline: https://github.com/EmaroLab/armor
- Install smach: http://wiki.ros.org/smach
- Download the topological map ``` git clone https://github.com/Imdimark/topological_map``` under the project folder( ```roscd EXPROLAB_Assignment2``` )

First running of the code:
- If the roscore is not running let's do: ```roscore & ```
- Go under the workspace ```cd <myworkspace>/ ```
- Build the workspace ```catkin make ```
- Finally is possible to launch the code ```roslaunch assignment1 assignment1.launch  ```

Running of the code:
- Check if the rocore is running, otherwise do step 1 of the previous paragraph
- launch the code ```roslaunch assignment1 assignment1.launch  ```


**Optional**, if you want to edit the topological map, using protegé install the editor following this guideline: https://protege.stanford.edu/

In this last case be careful that if you save the edits made through ARMOR you can have some errors, please follow this guide: 
Due to an internal bug of Protégé, when you save an ontology manipulated with aRMOR, and then try to open it with Protégé, you might face an error similar to `Cause: Don't know how to translate SWRL Atom: _:genid2147483693`. If this occurs you will not be able to open the ontology. However, you can open it by following these steps.

1. Open the ontology as a text file.
2. Search for `// Rules` (which should be the last section of the file).
3. Delete the `Rules` section. Be careful that the `</rdf:RDF>` statement should remain as the last line of the file.
4. Save the ontology from the text editor.
5. Now you can open the ontology with Protégé.

Note that the 3rd point deletes all the SWRL rules from the ontology. Hence, the inferences of the reasoner are different from the ones that you got within ROS. To fully visualise the reasoner inferences from Protégé you can proceed in two ways.

1. Open the original ontology with Protégé, and copy-paste the three `SWLRule` (from the `SWRL` tab) back into the ontology manipulated with aRMOR. Now you can update the reasoner and check the knowledge it infers through Protégé.
2. Make aRMOR save the ontology by exporting the `INFERENCES` (there is a dedicated command for it). In this way, you do not have to copy-paste back the rules in the ontology as in point 1. However, in this way, Protégé would not let you see any differences between the knowledge you asserted in the ontology, and the knowledge inferred by the reasoner.



## Working hypothesis and environment <a name="hyp"></a>

### System's features
The system uses ROS nodes and components like ARMOR and SMACH implements a very well finite state machine that automatically chooses the correct state and uses the ontology for the knowledge ( in this case about the environment). ROS plays a fundamental connecting all the players and giving the possibility to scale up or scale down the system, also watching future improvements.
The package is able to move the camera on top of the arm with Moveit and detect aruco markers using the aruco package and OpenCV for building ontology. The agent (robot) is able to implement the concept of a surveillance robot using gmapping and move_base. 


## System's limitations
Robots move very slowly due to bad balancing in the urdf (wheels are too close) and the camera works badly requiring two rotations to assuring the scansion. The battery mechanism works very well, but the robot reaches the charging station ( in corridor E) without using the remaining battery. This is very unrealistic.

## Possible technical improvements
One possible improvement is to implement an algorithm that preserves the battery to make it possible for the robot to reach the charging station with residual charge giving a more realistic behavior. Next to that, a future improvement will be constructing a better urdf (for example increasing the wheeles distance or adding two wheels). In this project instead of the simple joint controller, has been implemented MoveIt for giving it (in the future) the possibility to go in specificated points and scan harder aruco markers. This is more flexible for future improvements.

## Authors and contacts <a name="contacts"></a>

Author: Giovanni Di Marco
Mail: giovannidimarco06@gmail.com
Linkedin: https://www.linkedin.com/in/giovanni-di-marco-068453b1/


- Start the patrolling algorithm by relying on autonomous navigation strategies (mapping/planning) and on the information collected and stored in the ontology during the previous step.
- When a room is reached, perform a complete scan of the room (by rotating the base or the camera).



