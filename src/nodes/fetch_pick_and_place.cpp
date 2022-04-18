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

  // Ros Service Member Variables
  ros::ServiceClient clearOctomap;
  std_srvs::Empty srv;

  // Clear the octomap in case it was previously occupied
  clearOctomap = nh.serviceClient<std_srvs::Empty>("/clear_octomap");

  string planning_group;
  nh.getParam("/move_group/planning_group", planning_group);

  Manipulation manipulation(nh, planning_group);
  Manipulation arm_torso(nh, "arm_with_torso");

  manipulation.removeCollision("head_box");
  Navigation nav(nh);

  Perception perception(nh);
  Grasp_Cluster grasp_cluster(nh);

  ROS_WARN("Starting...");

  while(ros::ok())
  {
    arm_torso.place("retract");
    ros::Duration(1).sleep();

    manipulation.setGripper(1);
    nav.sendGoal(-0.680366873741, -0.661050796509, 0.7864987);
    nav.sendGoal(-1.05996143818, 0.711649179459, 2.3564987);
    nav.setHead(); // Temp: Move Fetch's head so it can see the table
    manipulation.addCollisionObjects();

    ros::Duration(2).sleep();

    perception.multiCameraSnapshot(nh);
    perception.publishCombinedCloud(perception.concatenateClouds(perception.cloud_list));
    while (!grasp_cluster.planning_grasp && ros::ok()) {
      perception.multiCameraSnapshot(nh);
      perception.publishCombinedCloud(perception.concatenateClouds(perception.cloud_list));
    }
    
    // Perform Pick & Place
    bool success = manipulation.pickandPlace(manipulation.createPickingEEFPoseList(grasp_cluster.get_grasp_candidates()),"place");

    // set planning flag to OK for next loop
    grasp_cluster.set_planning(0);

    if(success)
    {
      nav.sendGoal(-1.04679000378,-0.0925005674362, -2.2717481);
      manipulation.place("place");
      manipulation.setGripper(1);
      ros::Duration(1.0).sleep();
      manipulation.removeCollision("head_box");
      arm_torso.place("retract");
    }
    ros::Duration(1.0).sleep();
    manipulation.getParams(nh);
    }  

  return 0;
}
