# Experimental robotics laboratory - Assignment2

## Index of contents:
1. [Introduction](#introduction)
2. [Behaviour](#video)
3. [Software architecture](#sofar)
4. [Installation and running procedure](#installation)
5. [Working hypothesis and environment](#hyp)
6. [Authors and contacts](#contacts)




## Introduction <a name="introduction"></a>
The project at hand focuses on the development and implementation of a surveillance robot, engineered to operate within an indoor environment. The environment consists of a 2D layout with four rooms and three corridors. For an initial starting point, the robot is placed in a specific location referred to as 'E'.

<img src="https://github.com/Imdimark/SmachRobot_ROS/assets/78663960/471cb60b-42c2-490f-9482-4c0e266a9d8f" width="200" height="200">

The operational structure of the robot is based on a topological map, which it constructs using incoming data about the connections between various rooms, corridors, and doorways. This information aids the robot in maneuvering around its environment and effectively fulfilling its surveillance duties.
The robot's actions follow an endless cycle: it moves to a new location, waits for a while, and then moves again. This process continues until the robot's energy levels run low, at which point it returns to location 'E' for a recharge before resuming its routine.
When the robot's battery is not low, it navigates through the environment with a specific policy in mind. It's programmed to stay primarily in the corridors, keeping a closer eye on them. However, if a nearby room hasn't been inspected for a while, the robot will deviate from its path to check the room, ensuring complete and thorough surveillance.

## Behaviour <a name="video"></a>



https://github.com/Imdimark/assignment2/assets/78663960/b07a4abc-9370-47d6-8ba6-0cf3c8a66005


This video shows how the state machine works and goes through all the states. The window in the middle represents graphically how the smach machine works and which is the actual state. The other four windows, made by gnome terminals represent the four nodes that are running:

- **state of the battery** that is going up or down depending if it is discharging or charging (different state machine states)
- **movements** that shows the state of the movements like where the robot is, where the robot chan goes, and if the movements have been done
- **ontology** this node is in charge to contact the armor service the ontology and initialize the map and the visited time of the rooms (useful for the urgency threshold), the windows show up when the process is done and the fsm can proceed to the next state.
- **FSM** is the core of the system, this window shows up the changing of the states and some info. To better understand is suggested to follow state changes throughout the smach_viewer. 


## Software architecture <a name="sofar"></a>

### State Viewpoint

![UML drawio](https://github.com/Imdimark/SmachRobot_ROS/assets/78663960/d8306a3a-8e4d-4c79-a1b3-f12376af0b95)

### Node diagram
![rosgraph](https://github.com/Imdimark/assignment2/assets/78663960/6fb8f6b4-4baf-43d9-9ac5-132abbc27285)


## Installation and running procedure <a name="installation"></a>
Some generic requirements can be necessary(like Python and ros), but the suggestion is to start with this container (based on Linux):  [carms84/exproblab](https://hub.docker.com/r/carms84/exproblab) 

Some mandatory prerequisites are needed:
- Download the project in your workspace:
  - ```cd <myworkspace>/src/```
  - ```git clone https://github.com/Imdimark/SmachRobot_ROS```
- Install gnome terminal: ```sudo apt-get install gnome-terminal```
- Install armor, following this guideline: https://github.com/EmaroLab/armor
- Install smach: http://wiki.ros.org/smach
- Download the topological map ``` git clone https://github.com/Imdimark/topological_map``` under the project folder( ```roscd SmachRobot_ROS``` )

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

## System's limitations
Conformation of the room must be declared a prior (like doors, and rooms) and it could be more agnostic. The battery mechanism works very well, but the robot reaches the charging station ( in corridor E) without using the remaining battery. This is very unrealistic.

## Possible technical improvements
A possible technical improvement be implemented a simulating 3d environment environment like gazebo and Copelliasim. Next to that could be possible to implement an algorithm that preserves the battery to make it possible for the robot to reach the charging station. Regarding the agnosticism of the environment in the code, reaching information through a sensor like a camera could be useful to build the ontology before starting.

## Authors and contacts <a name="contacts"></a>

Author: Giovanni Di Marco
Mail: giovannidimarco06@gmail.com
Linkedin: https://www.linkedin.com/in/giovanni-di-marco-068453b1/






---------------------------------------------------------------------------------------------------------------------------------------------



![Screenshot 2023-07-14 100123](https://github.com/Imdimark/SmachRobot_ROS/assets/78663960/7cb4dc51-8ce3-4f5f-a7c0-62932a981d32)




miglioramenti futuri:fare in modo che arrivi alla stazione di ricarica con la batteria residua

In order to implement the mainly in corridor behaviour, the urgent threshold in the ontology has been changed.

















#################################################################################################################################










In the second assignment of the Experimental Robotics Laboratory course, you are requested to integrate the architecture developed in the first assignment with a robotic simulation.

To start, you are provided with this package, which contains:
- the definition of a custom message and a custom service
- a simulation environment representing the "house" to be monitored
- a node that implements a service: it requires the id (marker) detected by the robot and it replies with the information about the corresponding room (name of the room, coordinates of the center, connections with other rooms)
- A launch file, which starts Gazebo with the simulation environment, and the service node (assignment.launch).

You have to:
- Add a robot to the environment;
- Integrate (if needed, modify it) the architecture that you have developed in the first assignment to the given scenario.

In particular, the robot will have to:
- Be spawned in the initial position x = -6.0, y = 11.0
- Build the "semantic" map of the environment by detecting, without moving the base of the robot, all seven markers that are present around it, by calling the provided service node. Try to "scan" the environment in a comprehensive way, possibly exploring different solutions related to the robot's model. 
- Start the patrolling algorithm by relying on autonomous navigation strategies (mapping/planning) and on the information collected and stored in the ontology during the previous step.
- When a room is reached, perform a complete scan of the room (by rotating the base or the camera).




The robot is designed to primarily operate within the corridors, dedicating 80% of its operational time, or 720 seconds, to moving and performing functions in these areas. This aligns with its nature as a corridor monitoring or surveillance device.

When not in the corridors, the robot spends the remaining 20% of its time, or 180 seconds, visiting and checking the rooms. This visit includes moving from the corridor area to the room, checking the room's conditions, and returning to the corridor.

The robot is equipped with a battery with an autonomy of 900 seconds (15 minutes). Once the battery is completely drained, the robot autonomously returns to the charging station. This journey does not entail any energy consumption. After the battery has been fully charged, the robot resumes its activities by returning to the corridors or room areas as scheduled.

To ensure that all rooms are regularly visited and checked, the robot operates with an urgency threshold of 2160 seconds (36 minutes). This means that each room is visited and checked at least once within this time interval.

Moreover, the robot is programmed to return to the point where it interrupted its path after recharging, allowing it to resume its activities without interruptions.

This behavior ensures that the robot can perform its activities efficiently, adhering to energy limitations and operational priorities, while ensuring a regular check of all rooms.


Dipendenze:

https://github.com/ros-perception/vision_opencv 
aruco_ros package
my_robot_package
assignment2_package
