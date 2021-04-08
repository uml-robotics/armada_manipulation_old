// ********************************************************************************************
// Author: Brian Flynn;
// Test Engineer - NERVE Center @ UMASS Lowell
// basic_manipulation_and_2d_perception.cpp
//
// manipulation_class & perception_class node
//
// Utilizing the simulated ARMada workstation and a simulated robot, use any vision systems available
// to perform a very simple task - move the robot to a few set positions (created in the robot's moveit
// config package in its SRDF) and then concatenate the pointclouds and publish the result on a new topic
// This node focuses on 2D vision using openCV
// ********************************************************************************************

#include "manipulation_class.hpp"
#include "perception_class.hpp"
#include "grasp_cluster_class.hpp"

int main(int argc, char** argv)
{
  // Initialize node & spinner
  ros::init(argc,argv,"basic_manipulation_and_2d_perception_node");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(4);
  spinner.start();
  ros::Duration(1.0).sleep();

  // Instantiate and retrieve planning group from parameters, this should be set outside the node
  string planning_group;
  if (nh.getParam("/planning_group", planning_group))
  {
    ROS_INFO("Planning group is: %s ...", planning_group.c_str());
  } else
  {
    ROS_INFO("Failed to retrieve planning group parameter ...");
  }

  // Create manipulation, perception and grasping objects
  Manipulation manipulation(nh, planning_group);
  Perception perception(nh);
  Grasp_Cluster grasp_cluster(nh);

  // Set planning variables and reset robot position
  manipulation.move_group_ptr->setPlanningTime(15.0);
  manipulation.move_group_ptr->setMaxVelocityScalingFactor(0.25);
  manipulation.move_group_ptr->setPlannerId("RRTConnect");

  //manipulation.move_group_ptr->setNamedTarget("preset_1");
  manipulation.move_group_ptr->setNamedTarget("retract");
  manipulation.move_group_ptr->move();
  
  while(ros::ok())
  {
    // Put some code in here!
    // The purpose of this node is to use 2d vision to accomplish a simple pick and place task
    // You will probably want to take a snapshot of the system, publish the images, and then do some processing in another node
    // After the images have been processed, you can use this main node again to tell the robot to perform some action
  }  

  ros::waitForShutdown();
  return 0;

}
