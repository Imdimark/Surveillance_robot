![rosgraph](https://github.com/Imdimark/assignment2/assets/78663960/6fb8f6b4-4baf-43d9-9ac5-132abbc27285)# Assignment2



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
