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
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(4);
  spinner.start();  
  ros::Duration(1.0).sleep();

  string planning_group;
  nh.getParam("/planning_group", planning_group);

  Manipulation manipulation(nh, planning_group);
  Perception perception(nh);
  Grasp_Cluster grasp_cluster(nh);

  ros::Duration(5.0).sleep();

  while(ros::ok())
  {
    perception.multiCameraSnapshot(nh);
    perception.publishCombinedCloud(perception.concatenateClouds(perception.cloud_list));
    while (!grasp_cluster.planning_grasp && ros::ok()) {
      perception.multiCameraSnapshot(nh);
      perception.publishCombinedCloud(perception.concatenateClouds(perception.cloud_list));
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
