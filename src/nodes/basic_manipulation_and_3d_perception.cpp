// ********************************************************************************************
// Author: Brian Flynn;
// Test Engineer - NERVE Center @ UMASS Lowell
// basic_manipulation_and_3d_perception.cpp
//
// manipulation_class & perception_class node
//
// Utilizing the simulated ARMada workstation and a simulated robot, use any vision systems available
// to perform a very simple task - move the robot to a few set positions (created in the robot's moveit
// config package in its SRDF) and then concatenate the pointclouds and publish the result on a new topic
// // This node focuses on 3D vision using PCL
// ********************************************************************************************

#include "manipulation_class.hpp"
#include "perception_class.hpp"

int main(int argc, char** argv)
{
  // ros initialization
  ros::init(argc,argv,"basic_manipulation_and_3d_perception_node");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(4);
  spinner.start();  
  string planning_group = "manipulator_and_endeffector";

  // Wait for spinner to start
  ros::Duration(2.0).sleep();

  // Create manipulation and perception objects
  Manipulation manipulation(nh, planning_group);
  // ROS_INFO("planning group is: %s", planning_group);

  // Instantiate perception object
  Perception perception(nh);

  // Transform listener
  perception.transform_listener_ptr = TransformListenerPtr(
      new tf::TransformListener());

  // instantiate subscribers after creating transform listener
  perception.init_subscriber(nh);

  // Planning scene interface
  manipulation.planning_scene_ptr = PlanningScenePtr(
      new moveit::planning_interface::PlanningSceneInterface());

  // Moveit interface
  manipulation.move_group_ptr = MoveGroupPtr(
      new moveit::planning_interface::MoveGroupInterface(manipulation.PLANNING_GROUP));
  
  // Set useful variables before robot manipulation begins
  manipulation.move_group_ptr->setPlanningTime(15.0);			// This will give the robot the opportunity to plan for a while
  manipulation.move_group_ptr->setMaxVelocityScalingFactor(0.25);	// This will limit the robot to moving at 1/4 its max speed (they can go very fast)
  manipulation.move_group_ptr->setPoseReferenceFrame("world");		// We want to establish that the common reference frame is "world" for any path planning
  manipulation.move_group_ptr->setPlannerId("RRTConnect");		// There are numerous default planners, you can experiment or research which will suit your needs

  // take snapshots
  perception.collect_camera_snapshots();

  ros::Duration(1).sleep();

  // concatenate clouds
  perception.concatenate_clouds();
  ros::Duration(1).sleep();

  // publish concatenated cloud
  perception.publish_combined_cloud();

  while(ros::ok())
  {

  }  

  ros::waitForShutdown();
  return 0;

}
