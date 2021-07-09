// ********************************************************************************************
// Author: Brian Flynn
// NERVE Center @ UMASS Lowell
// pick_and_place.cpp
//
// pick and place node
//
// TODO: update this description after code is complete
// ********************************************************************************************

#include "manipulation_class.hpp"
#include "perception_class.hpp"
#include "grasp_cluster_class.hpp"

int main(int argc, char** argv)
{
  // Initialize node & check for planning_group arg
  ros::init(argc,argv,"pick_and_place_node");
  if (argc != 3) {
    ROS_INFO("Usage: pick_and_place <planning_group> <camera_type>");
    return 1;
  }
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(4);
  spinner.start();  
  ros::Duration(1.0).sleep();

  // Create objects for manipulation, perception and grasping operations
  string planning_group = argv[1];
  Manipulation manipulation(nh, planning_group);
  Perception perception(nh);
  Grasp_Cluster grasp_cluster(nh);
  
  manipulation.move_group_ptr->setPlanningTime(15.0);
  manipulation.move_group_ptr->setMaxVelocityScalingFactor(0.25);
  manipulation.move_group_ptr->setPlannerId("RRTConnect");
  manipulation.move_group_ptr->setNamedTarget("retract");         // preset_1 retract
  manipulation.move_group_ptr->move();

  ros::Duration(5.0).sleep();

  while(ros::ok())
  {
      // take snapshot and publish resulting concatenated pointcloud
      // check whether to use wrist or workstation cameras
      if (!strcmp(argv[2], "wrist")) {
          perception.generate_wrist_pointcloud();
      }
      else if (!strcmp(argv[2], "workstation")){
          perception.generate_workspace_pointcloud();
      }
      else if (!strcmp(argv[2], "base")){
          perception.generate_base_cam_pointcloud();
      }
      else {
          ROS_INFO("<camera_type> should be either wrist, workstation or base");
          ROS_INFO("Please ensure you entered the correct value for your device(s)");
          return 1;
      }

      ros::Duration(5.0).sleep();

      while (!grasp_cluster.planning_grasp && ros::ok()) {
          ros::Duration(5.0).sleep();
          perception.generate_workspace_pointcloud();
      }

      // Store grasp pose values and create a list of picking poses
      manipulation.store_gpd_vals(grasp_cluster.get_grasp_candidates());
      manipulation.createPickingEEFPoseList();

      // Perform Pick & Place
      manipulation.pick_and_place(manipulation.graspPoseList, manipulation.place_pose);

      // set planning flag to OK for next loop
      grasp_cluster.set_planning(0);
      ros::Duration(5.0).sleep();
  }  

  return 0;
}
