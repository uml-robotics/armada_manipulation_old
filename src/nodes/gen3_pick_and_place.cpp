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
  // Initialize node
  ros::init(argc,argv,"pick_and_place_node");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(4);
  spinner.start();
  ros::Duration(1.0).sleep();

  // Create objects for manipulation, perception and grasping operations
  string planning_group, wrist_cam;
  nh.getParam("/planning_group", planning_group);
  nh.getParam("/camera_names", wrist_cam);
  Manipulation manipulation(nh, planning_group);
  Perception perception(nh);
  Grasp_Cluster grasp_cluster(nh);
  ROS_INFO("Camera Topic: %s", wrist_cam);

  // Move to snapshot poses and add to pointcloud
  manipulation.move_group_ptr->setPlanningTime(15.0);
  manipulation.move_group_ptr->setMaxVelocityScalingFactor(0.25);
  manipulation.move_group_ptr->setPlannerId("RRTConnect");
  manipulation.move_group_ptr->setNamedTarget("retract");         // preset_1 retract
  manipulation.move_group_ptr->move();
  ros::Duration(5.0).sleep();
  //  manipulation.move_group_ptr->setNamedTarget("top_snapshot");
  //  manipulation.move_group_ptr->move();
  //  ros::Duration(5.0).sleep();
  //  perception.wristCameraSnapshot(nh, wrist_cam);
  manipulation.move_group_ptr->setNamedTarget("right_snapshot");
  manipulation.move_group_ptr->move();
  ros::Duration(5.0).sleep();
  perception.wristCameraSnapshot(nh, wrist_cam);
  while (perception.cloud_list[0].size() == 0) {
    // wait
  }
  manipulation.move_group_ptr->setNamedTarget("left_snapshot");
  manipulation.move_group_ptr->move();
  ros::Duration(5.0).sleep();
  perception.wristCameraSnapshot(nh, wrist_cam);
  while (perception.cloud_list[1].size() == 0) {
    // wait
  }
  ROS_INFO("cloud_list size: %d", perception.cloud_list.size());
  perception.publishCombinedCloud(perception.concatenateClouds(perception.cloud_list));
  // Store grasp pose values and create a list of picking poses
  manipulation.store_gpd_vals(grasp_cluster.get_grasp_candidates());
  manipulation.createPickingEEFPoseList();

  // Perform Pick & Place
  manipulation.pick_and_place(manipulation.graspPoseList, manipulation.place_pose);

  // set planning flag to OK for next loop
  grasp_cluster.set_planning(0);
  ros::Duration(5.0).sleep();

  return 0;
}
