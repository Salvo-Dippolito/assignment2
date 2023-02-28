# Finite State Machine for a Surveilance Robot v. 1.5

[Code Documentation](https://salvo-dippolito.github.io/assignment2/)
## Introduction
This code was developed as a solution to the second assignment of the Experimental Robotics Lab course. It was requested to create in a ROS environment a finite state machine architecture for a robot tasked with surveling the floor of a building. To reason about its postion in the floor's environment and to evaluate which rooms to surveil, the robot is introduced as an agent in an ontology with which it interacts by using the [armor package](https://github.com/EmaroLab/armor). The finite state machine architecture was instead stuructured by using [the SMACH package](http://wiki.ros.org/smach).

The robot should move around its environment giving priority to the rooms it hasn't visited for more than a certain threshold of time set by the user, the rooms that satisfy this condition are considered 'urgent'. The robot agent also has a simulated battery charging and discharging cycle that triggers it to stop executing its tasks so that it can move to a specific room E where it can plug in and recharge.

This repository contains version 1.5 of this project, version 1.0 is instead found in [this other repository](https://github.com/Salvo-Dippolito/ontological_surveyor)

## Software Architecture
This project depends on one main node, two action servers, the armor service and a robot state notifications node. The details of how these components work together are shown in this project's [documentation](https://salvo-dippolito.github.io/assignment2/).

#### version 1.5 updates

A few changes have been made compared to the [previous version](https://salvo-dippolito.github.io/ontological_surveyor/) of this project:

* A new topic state/reached_charging_point has been added so that the robot_state node can know from the finite state machine that the charging station  as been reached.

* The Choose Move state doesn't use an action server anymore but the Surveil Room state instead now does,since the robot is actually doing something that can be interrupted during the Surveil Room state. The Choose Move action server has been removed since an additional action server proved to be computationally cumbersome for ROS, dragging down the simulation performance. The Choose Move State is now very similar in structure to the Load Map state, with only one output to the Execute Move state.

* To move the arm in the Surveil Room state, the surveil_room_server sets itself as a publisher on the /arm_controller/command topic, where it sends the desired joint configurations for the robot's arm.

* The MoveBase action server is now getting called by the ExecuteMove action server so that the ExecuteMove state can actually move the robot in the environment. Interaction with this new acton server is still handled by the AgentState class which is now initializing three clients (to MoveBase, SurveilRoom and ExecuteMove) and two subscribers, respectively to the /state/battery_low and /state/charging_point_reached topics.

* To read ARUCO markers from the robot's camera and to later obtain information from these markers' ID tag, two additional services have been implemented:
    marker_id_service and marker_server. They are both c++ nodes. The first listed, marker_id_service, is a trigger service that starts executing once a trigger request gets sent by the state machine. It is tasked to move the arm to its hard coded positions so that it can then process the raw image message to detect if any markers are present. 

* Added the read_room_create_ontology() object to the HandleOntology class so that the initial state can run it instead of the create_ontology() object
used in this project's previous version. In read_room_create_ontology the arm is controlled to scan some ARUCO markers with a camera and extract ontology information from them, in create_ontology instead the usser was asked to set up the information himself.


### UML structure

This is the updated project's UML diagram:

![UML_corretto](https://user-images.githubusercontent.com/63560239/218745054-cf5caa3d-0331-4a60-9b6b-dfe163237be7.jpg)


The state machine node subscribes to the robot state node through the use of an interface class called AgentState. Through an instance of this class the state machine node is also set up as a client to the two action servers /surveil_room_server and /execute_move_server and as publisher on the /state/reached_charging_point topic. Almost all nodes (except the robot_state and the surveil_room nodes) are set up as clients to the armor service, the state machine node gets set as a client through the use of a class called HandleOntology and publishes on /arm_controller/command topic through an object of HandleOntology called read_room_create_ontology(). This object also sets the robot as a client to the /get_marker_id and /room_info server servers. 
Also the SurveilRoom action server publishes on /arm_controller/command topic to control the robot arm.

One last modification from the previous version is that the ExecuteMove action server now also takes room coordinates as part of its goal request.


### State Machine Structure

<img
    src="/images/statemachine.jpg"
    title="Project State Machine"
    width="75%" height="75%">

There is an initial state called Load Map which only gets accessed at the start of the simulation. In this updated version of this project, during this state, the robot moves its camera on a 5 DOF arm to view some ARUCO markers placed around the spawning location of the simulated world. These markers contain identification numbers that can be sent to a service to get information about the floor ontology.

The robot then enters its main routine which has been subdivided in three main actions and consequently three different states: choosing where to go (Choose Move), getting there (Execute Move), and finally surveiling the place for a while (Surveil Room). If at any moment during the execution of these states the robot state node signals that the battery is low, then the state machine would transition immediately into its charging state (Charging). 

As shown in the documentation, the Charging state is actually comprised of a two-phased task: moving back to the charging station and then waiting for the battery to charge. In this state the /execute_move_action_server is called with the chosen location preset as room 'E'.

In this next gif it's possible to see what is shown to the user when the Surveil Room state is interrupted by the Charging state: 
![](https://github.com/Salvo-Dippolito/assignment2/blob/master/images/interrupt_surveil.gif)


## Installation and Running Procedure

### Installation 

These are the steps to follow to get this simulation to run on your machine:
1. Clone this repository inside your workspace (make sure it has been sourced in your .bashrc).
2. Install the [ARMOR](https://github.com/EmaroLab/armor/issues/7) and [Smach-ROS](http://wiki.ros.org/smach/Tutorials/Getting%20Started) packages.
  User note: To interact with ARMOR a slightly modified version of the [armor_api](https://github.com/EmaroLab/armor_py_api) has been used with almost no modifications. To simplify the installation process, all the relevant scripts, along with the functions created ad hoc for this application (namely: disjoin_all_inds() and check_if_ind_b2_class() ) are present in this repository in the [armor_api](https://github.com/Salvo-Dippolito/assignment2/tree/main/scripts/armor_api) sub-folder. 
3. Install the [open_cv](https://github.com/ros-perception/vision_opencv) and [cv_bridge](https://github.com/ros-perception/vision_opencv/tree/noetic/cv_bridge) packages. Make sure to also include the aruco marker models into your Gazebo models directory.
4. Install the gmapping and navigation packages:
```
sudo apt-get install ros-<ros-distro>-openslam-gmapping
```
```
sudo apt-get install ros-<ros-distro>-navigation
```
5. Run `chmod +x <file_name>` for each pyhton file inside the scripts and armor_api folders.
6. Run `catkin_make` from the root of your workspace.
7. In order to correctly view this project's user interface you'll also need to install the `simple_colors` package by copying the following line on your terminal:

```
pip3 install simple-colors
```
6. xterm might also be needed for debugging purposes:
```
sudo apt-get -y install xterm
```


### Running the Code

After completing the installation procedures you can run the simulation by launching:

```
roslaunch assignment2 assignment.launch
```

```
roslaunch assignment2 ontology.launch
```

```
roslaunch assignment2 surveyor.launch
```


## System's Features

The user can define after how many seconds since the last visit should a room be visited again by the robot. Being able to control this 'urgencyThreshold' allows the robot to visit a larger number of rooms instead of repeatedly visiting the same few rooms that keep turning 'urgent' as soon as the robot has moved away from them.  
It is also possible, by changing in [this script](https://github.com/Salvo-Dippolito/assignment2/blob/main/scripts/robot_state.py) a boolean value from False to True, to switch the battery state notifier in the robot_state node from a predictable charging and discharging cycle to a random one. This last feature has been added just as a testing tool to test the system's behaviour when confronted with random stimuli.



## System's Limitations

 Only the most basic functions of the robot have been modelled so the only real disturbance to the robot's normal behavioural routine is the battery's change of state from high to low, this limits the state machine's behaviour to a fairly predictable routine. It would also be helpful to have a separate terminal that prints cleanly the robot's battery state and possibly give the option to the user to change the battery state at will, for now only the battery state can be displayed by modifying the code in robot_state.py and using xterm in the launch file. 

 The robot's navigation through the simulated world is completely dependent on the move_base planner whose parameters have been tuned but whose behaviour can be very inefficient. Possibly also due to it's size, the robot has some issues traversing doorways with this planner. It can take forever for the robot to reach a target and it takes around 30 seconds for the move_base action server to realise that the target has been reached and to update its state. 

 Lastly, since the marker positions have been hard coded in the script, the robot will struggle to recognize the placed markers if it gets spawned in a different place of the map or if the markers are moved.


## Future Improvements

The robot arm should be programmed to search for markers and scan them instead of moving to predisposed locations. A proper inverse kinematic algorithm 
could also be used instead of the current setup where joint angles are directly controlled.
The initial state could also read the markers more quickly by using a publisher-subscriber architecture instead of a service based architecture. 
In future iterations of this project a different planning algorithm can be used to try to overcome the issues described in the System's Limitations paragraph.

## Video Demonstration

A long video demonstration of this project in action is available on youtube at [this link](https://www.youtube.com/watch?v=hqCuiGLzAFw).
Skipping through this video we can see the various phases of the state machine described in this Readme. After a while the world's objects are moved around the robot to check the system's response to unpredictable changes. The last half hour of video was added both for enjoyment and to show how the system reacts to its main cores being shut down.

## Author's contacts

- Personal e-mail: salvo.dipp@gmail.com
- Institutional e-mail: s5324750@studenti.unige.it


