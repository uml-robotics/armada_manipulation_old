// ********************************************************************************************
// Author: Dan Hemphill
// NERVE Center @ UMASS Lowell
// gen3_pick_and_place.cpp
//
// Gen3 pick and place node
// ********************************************************************************************

#include "manipulation_class.hpp"
#include "perception_class.hpp"
#include "grasp_cluster_class.hpp"

int main(int argc, char** argv)
{
  // Initialize node
  ros::init(argc,argv,"gen3_2_pick_and_place_node");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(4);
  spinner.start();
  ros::Duration(1.0).sleep();

  // Ros Service Member Variables
  ros::ServiceClient clearOctomap;
  std_srvs::Empty srv;

  // Clear the octomap in case it was previously occupied
  clearOctomap = nh.serviceClient<std_srvs::Empty>("/clear_octomap");

  // Create objects for manipulation, perception and grasping operations
  string planning_group, wrist_cam;
  nh.getParam("/move_group/planning_group", planning_group);
  nh.getParam("/camera_names", wrist_cam);

  Manipulation manipulation(nh, planning_group);
  Perception perception(nh);
  Grasp_Cluster grasp_cluster(nh);

  ros::Duration(5.0).sleep();

  // Move to snapshot poses and add to pointcloud
  manipulation.place("right_snapshot");
  ros::Duration(4.5).sleep();
  perception.wristCameraSnapshot(nh, wrist_cam);
  while (perception.cloud_list[0].size() == 0) {
    // waitplace_custom
  }
  manipulation.place("left_snapshot");
  ros::Duration(4.5).sleep();
  perception.wristCameraSnapshot(nh, wrist_cam);
  while (perception.cloud_list[1].size() == 0) {
    // wait
  }

  // Publish cloud
  perception.publishCombinedCloud(perception.concatenateClouds(perception.cloud_list));

  // Store grasp pose values and create a list of picking poses
  while(grasp_cluster.candidates.grasps.size() == 0) {
    //wait
  }

  // Perform Pick & Place
  manipulation.pickandPlace(manipulation.createPickingEEFPoseList(grasp_cluster.get_grasp_candidates()), "retreat");

  // Set planning flag to OK for next loop
  grasp_cluster.set_planning(0);
  ros::Duration(5.0).sleep();

  return 0;
}
