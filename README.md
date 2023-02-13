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



# Finite State Machine for a Surveilance Robot
[Code Documentation](https://salvo-dippolito.github.io/assignment2/)
## Introduction
This code was developed as a solution to the first assignment of the Experimental Robotics Lab course. It was requested to create in a ROS environment a finite state machine architecture for a robot tasked with surveling the floor of a building. To reason about its postion in the floor's environment and to evaluate which rooms to surveil, the robot is introduced as an agent in an ontology with which it interacts by using the [armor package](https://github.com/EmaroLab/armor). The finite state machine architecture was instead stuructured by using [the SMACH package](http://wiki.ros.org/smach).

The robot should move around its environment giving priority to the rooms it hasn't visited for more than a certain threshold of time set by the user, the rooms that satisfy this condition are considered 'urgent'. The robot agent also has a simulated battery charging and discharging cycle that triggers it to stop executing its tasks so that it can move to a specific room E where it can plug in and recharge.

## Software Architecture
This project depends on one main node, two action servers, the armor service and a robot state notifications node. The details of how these components work together are shown in this project's [documentation](https://salvo-dippolito.github.io/assignment2/).

### UML structure

This is the project's UML diagram:

<img
    src="/images/UML.jpg"
    title="Project UML"
    width="75%" height="75%">
    
The state machine node subscribes to the robot state node through the use of an interface class called AgentState. Through an instance of this class the state machine node is also set up as a client to the two action servers /choose_move_action_server and /execute_move_action_server. All nodes except the robot_state node are set up as clients to the armor service, although the state machine node gets set up through the use of a class called HandleOntology.

### State Machine Structure

<img
    src="/images/statemachine.jpg"
    title="Project State Machine"
    width="75%" height="75%">

There is an initial state called Load Map which only gets accessed at the start of the simulation. In that state the floor ontology is created by following user specifications and the agent is placed in its charging room 'E'. The robot then enters its main routine which has been subdivided in three main actions and consequently three different states: choosing where to go (Choose Move), getting there (Execute Move), and finally surveiling the place for a while (Surveil Room). If at any moment during the execution of each of these states the robot state node where to notify that the battery level was too low then the state machine would transition immediately into its charging state (Charging). 

As shown in the documentation, the charging state is actually comprised of a two-phased task: moving back to the charging station and then waiting for the battery to charge. In this state the /execute_move_action_server is called with the chosen location preset as room 'E'.

In this next gif it's possible to see what is shown to the user when the Surveil Room state is interrupted by the Charging state: 
![](https://github.com/Salvo-Dippolito/assignment2/blob/main/images/interrupt_surveil.gif)


Here is instead shown an example of user interaction in the initial state Load Map:
![](https://github.com/Salvo-Dippolito/assignment2/blob/main/images/load_map.gif)

Side note: in the code and its documentation the verb 'surveil' has often been wrongfully substituted with the verb 'survey'.

## Installation and Running Procedure

### Installation 

These are the steps to follow to get this simulation to run on your machine:
1. Clone this repository inside your workspace (make sure it has been sourced in your .bashrc).
2. Install the [ARMOR](https://github.com/EmaroLab/armor/issues/7) and [Smach-ROS](http://wiki.ros.org/smach/Tutorials/Getting%20Started) packages.
  User note: To interact with ARMOR a slightly modified version of the [armor_api](https://github.com/EmaroLab/armor_py_api) has been used with almost no modifications. To simplify the installation process, all the relevant scripts, along with the functions created ad hoc for this application (namely: disjoin_all_inds and check_if_ind_b2_class) are present in this repository in the [armor_api](https://github.com/Salvo-Dippolito/assignment2/tree/main/scripts/armor_api) sub-folder.
3. Run `chmod +x <file_name>` for each pyhton file inside the scripts and armor_api folders.
4. Run `catkin_make` from the root of your workspace.
5. In order to correctly view this project's user interface you'll also need to install the `simple_colors` package by copying the following line on your terminal:

```
pip3 install simple-colors
```

### Running the Code

After completing the installation procedures you can run the simulation by launching:

```
roslaunch assignment2 surveyor.launch
```

## Working Assumptions

This initial version of the robot surveilance project is meant to barely put together the initial building blocks of what will be the final project for this year's course. By this reasoning, most of the software solutions applied here are just integrated as a proof of concept. The Choose Move state for example follows a particularly simple policy where it only has to reason over the rooms immediately reachable by the robot and doesn't have to plan ahead any movements through the ontology's connected locations. This implies that when the robot has to reach room 'E' from whichever room in the floor ontology, it doesn't have to build a logical plan to reach it but can just place the robot directly in 'E' without passing through any other room.

Also the Execute Move state uses a very simplified version of what the final version of the code will have to do. In fact, the distance it simulates to traverse is a random number between 0.5 and 4 meters, and the direction in which this distance is covered is completely ignored.
Moreover, as mentioned in the [documentation](https://salvo-dippolito.github.io/assignment2/), the layout of the floors that can be described by the ontology is stuctured so that all corridors connect to room 'E' and all subsequent corrdidors are always connected to each other. To better explain, if there are 5 corridors, corridor 3 will be connected to (aside from its rooms) room 'E', corridor 2 and corridor 4.
For more details on the use of individuals and rules used in the ontology, please refer to the [topological_map](https://github.com/buoncubi/topological_map) repository on which this project was based.

## System's Features

As mentioned, the system allows the user to specify how many corridors are on the floor that the robot has to surveil and how many rooms are connected to each of these corridors. The user can also define after how many seconds since the last visit should a room be visited again by the robot. Being able to control this 'urgencyThreshold' allows the robot to visit a larger number of rooms instead of repeatedly visiting the same few rooms that keep turning 'urgent' as soon as the robot has moved away from them.  
It is also possible, by changing in [this script](https://github.com/Salvo-Dippolito/assignment2/blob/main/scripts/robot_state.py) a boolean value from False to True, to switch the batttery state notifier in the robot_state node from a predictable charging and discharging cycle to a random one. This last feature has been added just as a testing tool to test the system's behaviour when confronted with random stimuli.

## System's Limitations

As mentioned in the Working Assumptions section, this current version of the system can't plan paths through a series of connected rooms but can only decide which of the reachable rooms should be visited on the basis of a very simple behaviour policy. Moreover, only the most basic functions of the robot have been modelled so the only real disturbance to the robot's normal behavioural routine is the battery's change of state from high to low, this limits the state machine's behaviour to a fairly predictable routine. 

## Future Improvements

Many aspects of this system can still be improved but were purposely left 'unresolved' so as to not over complicate the system, considering that it will soon undergo some major changes before reaching its final form. One of these things is the relation between the Charging state and the robot_state's battery status updater. In this version of the code, instead of waiting for a notification to be sent by the execute move action server to notify the battery state function when the robot has reached the charging station, the robot_state node just waits for a pre-set amount of time before initiating the robot's charging sequence. This works fine for now since the maximum distance that can be travelled by the robot during the execute move state is known, and in the worst case scenario there will just be a few seconds delay between when the robot reaches the station and when it actually starts charging.
The battery discharging cycle could also be linked to the distance travelled by the robot and not be an arbitrarily decreasing value as it is set for now. In general, many elements of this version of the code are just placeholders for some future modifications that eil instead consist of some actually functional code.

## Author's contacts

- Personal e-mail: salvo.dipp@gmail.com
- Institutional e-mail: s5324750@studenti.unige.it

