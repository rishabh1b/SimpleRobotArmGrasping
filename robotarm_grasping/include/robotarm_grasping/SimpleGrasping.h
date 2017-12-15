
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include "geometry_msgs/PoseStamped.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <cv_bridge/cv_bridge.h>
#include "robotarm_grasping/VisionManager.h"
#include <tf/transform_listener.h>

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