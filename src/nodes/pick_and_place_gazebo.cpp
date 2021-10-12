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
#include "uml_hri_nerve_pick_and_place/SpawnObject.h"

int main(int argc, char** argv)
{
  // Initialize node & check for planning_group arg
  ros::init(argc,argv,"pick_and_place_node");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(4);
  spinner.start();
  ros::Duration(1.0).sleep();

  // Initialize Octomap clearing service
  ros::ServiceClient clearOctomap = nh.serviceClient<std_srvs::Empty>("/clear_octomap");;
  std_srvs::Empty clearOctomapSrv;

  // Intialize Gazebo object spawning/deleting services
  ros::ServiceClient spawnModel = nh.serviceClient<uml_hri_nerve_pick_and_place::SpawnObject>("/spawn_object");
  uml_hri_nerve_pick_and_place::SpawnObject spawnModelSrv;
  ros::ServiceClient deleteModel = nh.serviceClient<gazebo_msgs::DeleteModel>("/gazebo/delete_model");
  gazebo_msgs::DeleteModel deleteModelSrv;

  string planning_group;
  nh.getParam("/move_group/planning_group", planning_group);

  Manipulation manipulation(nh, planning_group);
  Perception perception(nh);
  Grasp_Cluster grasp_cluster(nh);

  ros::Duration(5.0).sleep();

  while(ros::ok())
  {
    // clear octomap
    clearOctomap.call(clearOctomapSrv);
    ros::Duration(2.0).sleep();

    //Setup object spawning FIX THIS LATER
    std::vector<double> test_pose_x_list;
    std::vector<double> test_pose_y_list;
    test_pose_x_list.clear();
    test_pose_y_list.clear();
    test_pose_x_list.push_back(-0.1);
    test_pose_x_list.push_back(-0.1);
    test_pose_x_list.push_back(0.1);
    test_pose_x_list.push_back(0.1);
    test_pose_x_list.push_back(0.0);

    test_pose_y_list.push_back(-0.1);
    test_pose_y_list.push_back(-0.1);
    test_pose_y_list.push_back(0.1);
    test_pose_y_list.push_back(0.1);
    test_pose_y_list.push_back(0.0);

    int test_position = rand() % 5 + 1;

    double test_pose_x = {test_pose_x_list[test_position]};
    double test_pose_y = {test_pose_y_list[test_position]};


    string model_name = "coke_can";
    string file_path = "/home/brian/.gazebo/models/coke_can/model.sdf";
    geometry_msgs::Pose object_pose;
    object_pose.orientation.w = 1.0;
    object_pose.position.x = test_pose_x;
    object_pose.position.y = test_pose_y;
    object_pose.position.z = 1.2;

    // Create spawn model request
    spawnModelSrv.request.model_name = model_name;
    spawnModelSrv.request.file_path = file_path;
    spawnModelSrv.request.robot_namespace = "";
    spawnModelSrv.request.object_pose = object_pose;
    spawnModelSrv.request.reference_frame = "world";

    // Create delete model request (make sure names of objects are accurate
    deleteModelSrv.request.model_name = model_name;
    // Spawn object(s)
    if (spawnModel.call(spawnModelSrv)) {
      ROS_INFO_STREAM("Spawning model: " << spawnModelSrv.request.model_name);
    } else {
      ROS_ERROR("Failed to call service spawn_model");
      return 1;
    }
    ros::Duration(1.5).sleep();

    perception.multiCameraSnapshot(nh);
    perception.publishCombinedCloud(perception.concatenateClouds(perception.cloud_list));
    while (!grasp_cluster.planning_grasp && ros::ok()) {
      perception.multiCameraSnapshot(nh);
      perception.publishCombinedCloud(perception.concatenateClouds(perception.cloud_list));
    }

    // Perform Pick & Place
    manipulation.pickandPlace(manipulation.createPickingEEFPoseList(grasp_cluster.get_grasp_candidates()), "wait");

    // Set planning flag to OK for next loop
    grasp_cluster.set_planning(0);
    ros::Duration(3.0).sleep();

    // Remove object(s)
    if (deleteModel.call(deleteModelSrv)) {
      ROS_INFO_STREAM("Deleting model: " << deleteModelSrv.request.model_name);
    } else {
      ROS_ERROR("Failed to call service delete_model");
      return 1;
    }
    ros::Duration(1.5).sleep();

    // Update and assign parameters
    manipulation.getParams(nh);
  }

  return 0;
}
