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
  ros::init(argc,argv,"pick_and_place_gazebo_node");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(4);
  spinner.start();
  ros::Duration(1.0).sleep();

  // Initialize Octomap clearing service
  ros::ServiceClient clearOctomap = nh.serviceClient<std_srvs::Empty>("/clear_octomap");;
  std_srvs::Empty srv;

  // Intialize Gazebo object spawning/deleting services
  ros::ServiceClient spawnModel;

  string planning_group;
  nh.getParam("/move_group/planning_group", planning_group);

  Manipulation manipulation(nh, planning_group);
  Perception perception(nh);
  Grasp_Cluster grasp_cluster(nh);

  ros::Duration(15.0).sleep();

  while(ros::ok())
  {
    // reinitialize cloud list
    //grasp_cluster.set_planning(1);
    perception.cloud_list.resize(0);

    manipulation.moveNamed("robot_left");
    ros::Duration(6.0).sleep();
    perception.wristCameraSnapshot(nh, "wrist_camera");
    ros::Duration(0.5).sleep();

    manipulation.moveNamed("robot_right");
    ros::Duration(6.0).sleep();
    perception.wristCameraSnapshot(nh, "wrist_camera");
    ros::Duration(0.5).sleep();

    manipulation.moveNamed("wait");
    ros::Duration(6.0).sleep();
    perception.wristCameraSnapshot(nh, "wrist_camera");
    ros::Duration(0.5).sleep();

    //grasp_cluster.set_planning(0);
    perception.publishCombinedCloud(perception.concatenateClouds(perception.cloud_list));
    ros::Duration(5.0).sleep();

    // Perform Pick & Place
    manipulation.pickandPlace(manipulation.createPickingEEFPoseList(grasp_cluster.get_grasp_candidates()), "wait");

    // set planning flag to OK for next loop
    grasp_cluster.set_planning(0);
    ros::Duration(5.0).sleep();
    manipulation.getParams(nh);

  }

  return 0;
}
