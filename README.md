# SimpleRobotArmGrasping
In this project an attempt is being made to simulate a basic grasping application using a two finger gripper to grasp uniform objects like a cylinder.  The focus is to be able to come with an intial prototype of the arm that will be installed on a mobile manipulator. Typically, an object that lies on the table needs to be grasped. The camera on the top of the table provides a visual feedback on the location of the object in 2D. The location of the camera is known w.r.t to the Robot frame (by a technique called as Robot-Camera Calibration). I am assuming that the geometry of the object to be grasped is uniform (cylinder) and its height is known, so I will use the 2D camera information to move my manipulator to this point to grasp and lift this object. Although, this seems to be a machine vision application, the camera setting described here gives the initial proof of concept. This camera can be replaced by a 3D camera which would be attached to the robot’s body and this in turn will give the necessary 3D position of the object to be grasped. 

## Standard install via command-line
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/rishabh1b/SimpleRobotArmGrasping/
catkin_init_workspace
cd ~/catkin_ws
catkin_make
```
## Solo Iterative Process and Sprint Planning Notes
The development of this software is being undertaken through agile methodology in a three week sprint. This is the link for [SIP](https://docs.google.com/spreadsheets/d/1VbUxU0HfxbzXvX9tXwKXp6oL624SC5Jw42VlmQ8VTdk/edit?usp=sharing)

## Running a basic demo
*Details will be added soon*

## Dependencies
1. ROS Kinetic. This can be downloaded by following the steps [here](http://wiki.ros.org/kinetic/Installation).
2. CMake. This can be downloaded from [here](https://cmake.org/download/)
3. MoveIt!. This can called from [here](http://moveit.ros.org/install/). Make sure you Follow Instructions for ROS Kinetic

## Doxygen Documentation
*Details will be added soon*

## Known Issues/Bugs
