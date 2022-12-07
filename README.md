# Assignment2

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


