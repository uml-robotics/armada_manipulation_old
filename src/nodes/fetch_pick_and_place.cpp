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

  Logger log(nh);
  log.writeHeaders();

  
  // Ros Service Member Variables
  ros::ServiceClient clearOctomap;
  std_srvs::Empty srv;

  // Clear the octomap in case it was previously occupied
  clearOctomap = nh.serviceClient<std_srvs::Empty>("/clear_octomap");

  string planning_group;
  nh.getParam("/move_group/planning_group", planning_group);

  Manipulation manipulation(nh, planning_group, &log);
  Manipulation torso(nh, "arm_with_torso", &log);

  Navigation nav(nh, &log);
  Perception perception(nh);
  Grasp_Cluster grasp_cluster(nh);

  ROS_WARN("Starting...");
  torso.place("retract");
  manipulation.addCollisionObjects();
  nav.sendGoal(-0.680366873741, -0.661050796509, 0.7864987); // HOME

  while(ros::ok())
  {
    log.open();
    manipulation.removeCollision("head_box");
    torso.place("retract");
    manipulation.setGripper(1);

    log.startTime(); // home to pick time, nav

    nav.sendGoal(-1.060, 0.712, 2.356); // PICK 1
    //nav.sendGoal(-1.420, 0.460, 1.363); // PICK 2
    //nav.sendGoal(-0.773, 1.065, -2.986); // PICK 3

    log.endTime(); // home to pick time, nav
    log.data.time_home_pick = log.getDuration();

    ros::Duration(2).sleep();
    nav.setHead(); // Move Fetch's head so it can see the table
    manipulation.addCollisionObjects();
    torso.addCollisionObjects();

    while (!grasp_cluster.planning_grasp && ros::ok()) {
      ros::Duration(2).sleep();
      perception.multiCameraSnapshot(nh);
      perception.publishCombinedCloud(perception.concatenateClouds(perception.cloud_list));
    }
    
    // Perform Pick & Place
    bool success = manipulation.pickandPlace(manipulation.createPickingEEFPoseList(grasp_cluster.get_grasp_candidates()),"place");
    log.startTime(); // nav
    // set planning flag to OK for next loop
    grasp_cluster.set_planning(0);
    log.data.planning_success = success;
    if(success)
    {
      nav.sendGoal(-1.04679000378,-0.0925005674362, -2.2717481); // PLACE
      log.endTime(); // retract to place nav
      log.data.time_retract_place = log.getDuration();

      manipulation.addCollisionObjects();
      torso.addCollisionObjects();
      log.startTime(); // place to dropped, manip
      manipulation.place("place");
      manipulation.setGripper(1); // Open gripper
      ros::Duration(1.0).sleep();

      manipulation.removeCollision("head_box");
      torso.place("retract");
      log.endTime(); // place to dropped, manip
      log.data.time_place_dropped = log.getDuration();

      log.startTime(); // dropped to home, nav
      nav.sendGoal(-0.680366873741, -0.661050796509, 0.7864987); // HOME
      log.endTime(); // dropped to home, nav
      log.data.time_dropped_home = log.getDuration();
    }
    ros::Duration(1.0).sleep();
    manipulation.getParams(nh);
    log.endTime();
    log.writeData();
    log.close();
    }

    log.close();

  return 0;
}
