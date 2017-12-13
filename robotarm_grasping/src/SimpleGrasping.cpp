#include "robotarm_grasping/SimpleGrasping.h"

SimpleGrasping::SimpleGrasping(ros::NodeHandle n_, float length, float breadth)
    : it_(n_), group("arm"), vMng_(length, breadth)
{
    this->nh_ = n_;
    namespace rvt = rviz_visual_tools;
    // visual_tools("odom_combined");
	//visual_tools.deleteAllMarkers();

    // Subscribe to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/robot_arm/camera1/robot_arm/image_raw", 1,
      &SimpleGrasping::imageCb, this);

    // this->display_publisher = nh_.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);

    obj_found = false;
}

void SimpleGrasping::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
	if (!obj_found) {
	    ROS_INFO("Start Processing the Image");
	    try {
	      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	    }
	    catch (cv_bridge::Exception& e){
	      ROS_ERROR("cv_bridge exception: %s", e.what());
	      return;
		}

	    ROS_INFO("Image Message Received");
	    float obj_x, obj_y;
	    vMng_.get2DLocation(cv_ptr->image, obj_x, obj_y);

        // Temporary Debugging
	    std::cout<< " X-Co-ordinate in Camera Frame :" << obj_x << std::endl;
	    std::cout<< " Y-Co-ordinate in Camera Frame :" << obj_y << std::endl;

	    obj_found = true;
	}
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
  float length = 1;
  float breadth = 0.6;

  ros::NodeHandle n;
  SimpleGrasping simGrasp(n); 
  ros::spinOnce();
  simGrasp.attainPosition();
  return 0;
}