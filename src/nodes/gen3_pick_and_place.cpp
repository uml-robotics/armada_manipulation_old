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
  if (argc < 3) {
    ROS_INFO("Usage: pick_and_place <planning_group> <camera_topic_1> <camera_topic_2> ... <camera_topic_n>");
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

  // Generate a list of camera topics for perception object to iterate through for concatenating a dynamic number of clouds
  for (int i = 2; i < argc; ++i) {
    perception.camera_names.push_back(argv[i]);
    ROS_INFO("Camera Topic %d: %s", i-2, argv[i]);
  }

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
  //  perception.workstation_snapshot(nh);
  //  ros::Duration(1.0).sleep();
  manipulation.move_group_ptr->setNamedTarget("right_snapshot");
  manipulation.move_group_ptr->move();
  ros::Duration(5.0).sleep();
  perception.workstation_snapshot(nh);
  ros::Duration(1.0).sleep();
  manipulation.move_group_ptr->setNamedTarget("left_snapshot");
  manipulation.move_group_ptr->move();
  ros::Duration(5.0).sleep();

  ROS_INFO("Publishing combined pointcloud");
  while(ros::ok())
  {
      // take snapshot and publish resulting concatenated pointcloud
      perception.generate_workspace_pointcloud(nh);
      ros::Duration(1.0).sleep();

      while (!grasp_cluster.planning_grasp && ros::ok()) {
          perception.generate_workspace_pointcloud(nh);
          ros::Duration(1.0).sleep();
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
