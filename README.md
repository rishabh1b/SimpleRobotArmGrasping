# SimpleRobotArmGrasping
[![Build Status](https://travis-ci.org/rishabh1b/SimpleRobotArmGrasping.svg?branch=master)](https://travis-ci.org/rishabh1b/SimpleRobotArmGrasping)
## Overview
In this project an attempt is made to simulate a basic grasping application using a two finger gripper to grasp uniform objects like a cylinder.  The focus was to be able to come with an intial prototype of the arm that will be installed on a mobile manipulator. Typically, an object that lies on the table needs to be grasped. The camera on the top of the table provides a visual feedback on the location of the object in 2D. The location of the camera is known w.r.t to the Robot frame (by a technique called as Robot-Camera Calibration). I am assuming that the geometry of the object to be grasped is uniform (cylinder) and its height is known, so I have used the 2D camera information to move my manipulator to this point to grasp and lift this object. Although, this seems to be a machine vision application, the camera setting described here gives the initial proof of concept. This camera can be replaced by a 3D camera which would be attached to the robot’s body and this in turn will give the necessary 3D position of the object to be grasped. The robot platform is custom-designed using URDF and xacro. 

## Description of packages in this repository
There are in total four packages in this repository. These are described briefly as follows - 
1. **robotarm_description** - This package has the description of the Robot Model which was created for this project. Specifically, it has the URDF and the meshes of the links of the robot model. Different joints and links of this model can be tested using following command -
```
roslaunch robotarm_description display_xacro.launch model:=robotArm.xacro
```
2. **robotarm_gazebo** - This package has the simulation specifics related to gazebo such as the world files and the models. Basically, this package takes care of spwaning the robot and setting the Table and object for our purpose. Also, it has some joint state controllers plugins to control the joints externally
3. **robotarm_config** - This package has the moveit configuration files and the controllers to control the arm in gazebo. Bascially, the config files were generated using the moveit setup assistant. 
4. **robotarm_grasping** - This package has the main implementation of Robot Arm movement and grasping. The main node is ```simpleGrasping``` and it basically issues the control commands and queries the camera for object location


## Workflow
The camera at the top takes a snapshot of the object and the table. If this is the first time the node is being run, It tries detecting the table too and calculates the pixels per mm in x and pixels per mm in y. Once these fields are pouplated the grasping behaviour is triggered. Grasping first attains a safe position and them takes the postion of the object just detected by the camera to reach near the object. Then Prismatic joints of the gripper are triggered and grasping is attempted. Once the fripper is closed sufficiently, the arm moves up and goes back to the home position. 

## Standard install via command-line
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/rishabh1b/SimpleRobotArmGrasping/
catkin_init_workspace
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
catkin_make
```
## Solo Iterative Process and Sprint Planning Notes
The development of this software is being undertaken through agile methodology in a three week sprint. This is the link for [SIP](https://docs.google.com/spreadsheets/d/1VbUxU0HfxbzXvX9tXwKXp6oL624SC5Jw42VlmQ8VTdk/edit?usp=sharing).
Also, Sprint Planning notes are located [here](https://docs.google.com/document/d/1oRomzkn-AGI27AvpXqasaDczI-XDEATqI8c4x9QcAeI/edit?usp=sharing)

## Running a basic demo
```roslaunch robotarm_grasping graspingdemo.launch```

## Dependencies
1. ROS Kinetic. This can be downloaded by following the steps [here](http://wiki.ros.org/kinetic/Installation).
2. CMake. This can be downloaded from [here](https://cmake.org/download/)
3. MoveIt!. This can called from [here](http://moveit.ros.org/install/). Make sure you Follow Instructions for ROS Kinetic

## Doxygen Documentation
1. Detailed documentation can be found under ```/docs/html/index.html```. 
2. To generate documentation, run ```sudo apt-get install doxygen-gui```. This is basically a GUI for doxygen. Once installed, run 
```doxywizard``` on the command line to launch the application. Then four steps to generate the documention are self-explanatory as in the wizard.

## Demonstration
<a href="http://www.youtube.com/watch?feature=player_embedded&v=j-qDS4KmnUQ
" target="_blank"><img src="https://github.com/rishabh1b/SimpleRobotArmGrasping/blob/master/outputs/output1.png" 
alt="IMAGE ALT TEXT HERE" width="240" height="180" border="10" /></a>

## Known Issues/Bugs
1. Gripper does not open once closed
2. Arm does not have a perception of the table and hence it may collide at times.
3. Gripper is not able to grasp the object in the simulator. 
