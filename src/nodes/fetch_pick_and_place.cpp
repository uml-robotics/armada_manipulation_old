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
#include "navigation_class.hpp"
#include "perception_class.hpp"
#include "grasp_cluster_class.hpp"

int main(int argc, char** argv)
{
  // Initialize node & check for planning_group arg
  ros::init(argc,argv,"pick_and_place_node");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(4);
  spinner.start();
  ros::Duration(4).sleep(); // Temp fix: need time for Fetch's head point action to start

  // Ros Service Member Variables
  ros::ServiceClient clearOctomap;
  std_srvs::Empty srv;

  // Clear the octomap in case it was previously occupied
  clearOctomap = nh.serviceClient<std_srvs::Empty>("/clear_octomap");

  string planning_group;
  nh.getParam("/move_group/planning_group", planning_group);

  Manipulation manipulation(nh, planning_group);
  manipulation.setGripper(1);
  manipulation.place("retract");

  Navigation nav(nh);
  nav.sendGoal(-1.379, 0.266, 0);

  Perception perception(nh);
  Grasp_Cluster grasp_cluster(nh);

  ros::Duration(3.0).sleep();

  nav.sendGoal(0, 0, -44);
  ros::Duration(1).sleep();
  nav.setHead(); // Temp: Move Fetch's head so it can see the table
  manipulation.addCollisions();

  while(ros::ok())
  {
    perception.multiCameraSnapshot(nh);
    perception.publishCombinedCloud(perception.concatenateClouds(perception.cloud_list));
    
    while (!grasp_cluster.planning_grasp && ros::ok()) {
      perception.multiCameraSnapshot(nh);
      perception.publishCombinedCloud(perception.concatenateClouds(perception.cloud_list));
    }
    
    // Store grasp pose values and create a list of picking poses
    manipulation.storeGpdVals(grasp_cluster.get_grasp_candidates());
    manipulation.createPickingEEFPoseList();
    
    // Perform Pick & Place
    manipulation.pickAndPlace(manipulation.graspPoseList, "place");

    // set planning flag to OK for next loop
    grasp_cluster.set_planning(0);
    manipulation.place("retract");

    ros::Duration(5.0).sleep();
    manipulation.getParams(nh);
    
  }  

  return 0;
}
