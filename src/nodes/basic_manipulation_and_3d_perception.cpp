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
// This node focuses on 3D vision using PCL
// ********************************************************************************************

#include "manipulation_class.hpp"
#include "perception_class.hpp"
#include "grasp_cluster_class.hpp"

int main(int argc, char** argv)
{
  // Initialize node & spinner
  ros::init(argc,argv,"basic_manipulation_and_3d_perception_node");
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
      // take snapshot and publish resulting concatenated pointcloud
      perception.generate_workspace_pointcloud();
      //perception.generate_wrist_pointcloud();

      while (!grasp_cluster.planning_grasp)
      {
          // do nothing and wait
      }

      // give manipulation object gpd pose data
      manipulation.store_gpd_vals(grasp_cluster.get_grasp_candidates());

      // take first grasp candidate and create picking poses
      manipulation.createPickingEEFPose(manipulation.candidates.grasps[0]);

      // run testing pick and place operation with first grasp candidate pre/actual/after poses
      manipulation.pick_and_place(manipulation.grasp_poses);

      ros::Duration(2.0).sleep();

      // move back to a neutral position
      manipulation.move_group_ptr->setNamedTarget("retract");
      //manipulation.move_group_ptr->setNamedTarget("preset_1");
      manipulation.move_group_ptr->move();

      // set planning flag to OK for next loop
      grasp_cluster.set_planning(0);

      ros::Duration(1.0).sleep();
  }  

  ros::waitForShutdown();
  return 0;

}
