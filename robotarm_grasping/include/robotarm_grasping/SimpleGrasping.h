#pragma once
/**
* @file SimpleGrasping.h Class Declaration of Simple Grasping class
* @author rishabh1b(Rishabh Biyani)
* @copyright BSD 3-Clause License (c) Rishabh Biyani 2017
*/

// BSD 3-Clause License

// Copyright (c) 2017, Rishabh Biyani
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:

// * Redistributions of source code must retain the above copyright notice, this
// list of conditions and the following disclaimer.

// * Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and / or other materials provided with the distribution.

// * Neither the name of the copyright holder nor the names of its
// contributors may be used to endorse or promote products derived from
// this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_listener.h>
#include "robotarm_grasping/VisionManager.h"

class SimpleGrasping {
 private:
    ros::NodeHandle nh_;
    geometry_msgs::Pose target_pose1;
    moveit::planning_interface::MoveGroupInterface armgroup;
    moveit::planning_interface::MoveGroupInterface grippergroup;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;

	// For testing move_group
    bool grasp_running;

    cv_bridge::CvImagePtr cv_ptr;

	VisionManager vMng_;

	tf::StampedTransform camera_to_robot_;

	tf::TransformListener tf_camera_to_robot;

	tf::Vector3 obj_camera_frame, obj_robot_frame;

	geometry_msgs::PoseStamped homePose;

	moveit::planning_interface::MoveGroupInterface::Plan my_plan;

	float pregrasp_x, pregrasp_y, pregrasp_z;

	void attainPosition(float x, float y, float z);
	void attainObject();
	void grasp();
	void lift();

 public:
	SimpleGrasping(ros::NodeHandle n_, float pregrasp_x, float pregrasp_y, float pregrasp_z, float length = 1, float breadth = 0.6);
	void imageCb(const sensor_msgs::ImageConstPtr& msg);
	void initiateGrasping();
	void goHome();
};