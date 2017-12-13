#include "robotarm_grasping/SimpleGrasping.h"

SimpleGrasping::SimpleGrasping(ros::NodeHandle n_)
    : it_(n_), group("arm")
{
    this->nh_ = n_;
    namespace rvt = rviz_visual_tools;
    // visual_tools("odom_combined");
	//visual_tools.deleteAllMarkers();

    // Subscribe to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/robot_arm/camera1/robot_arm/image_raw", 1,
      &SimpleGrasping::imageCb, this);

    // this->display_publisher = nh_.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);

    tried_once = false;
}

void SimpleGrasping::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
    ROS_INFO("Start Processing the Image");
  	if (!tried_once)
  		attainPosition();

  	tried_once = true;
}

void SimpleGrasping::attainPosition() {
  ROS_INFO("The attain position function called");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  // sleep(2.0);

  // Raw pointers are frequently used to refer to the planning group for improved performance.
   const robot_state::JointModelGroup *joint_model_group =
	group.getCurrentState()->getJointModelGroup("arm"); 

  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("base_link_2");
  visual_tools.deleteAllMarkers();

  // We can print the name of the reference frame for this robot.
  ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
  
  // We can also print the name of the end-effector link for this group.
  ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());

  ROS_INFO("Group names: %s", 	group.getName().c_str());

  // For getting the pose
  geometry_msgs::PoseStamped currPose = group.getCurrentPose();
  std::cout<<group.getCurrentPose();

  geometry_msgs::Pose target_pose1;
  target_pose1.orientation = currPose.pose.orientation;
  //target_pose1.position.x = 0.4;
  //target_pose1.position.y = 0.04;
  //target_pose1.position.z = 0.32;

   // Starting Postion before picking
    target_pose1.position.x = 0.45;
    target_pose1.position.y = 0.09;
    target_pose1.position.z = 0.37;
  group.setPoseTarget(target_pose1);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = group.plan(my_plan);

  /*ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
  visual_tools.publishAxisLabeled(target_pose1, "pose1");
  // visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("next step");*/

  group.move();
  sleep(5.0);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "simple_grasping");
  ros::NodeHandle n;
  SimpleGrasping simGrasp(n); 
  // ros::spin();
  simGrasp.attainPosition();
  return 0;
}