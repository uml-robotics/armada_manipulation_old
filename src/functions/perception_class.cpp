// ********************************************************************************************
// Author: Brian Flynn;
// Test Engineer - NERVE Center @ UMASS Lowell
// perception_class.cpp
//
// perception_class function definitions
// ********************************************************************************************

#include "perception_class.hpp"

// ********************************************************************************************
// Constructors
// Create instance of Perception class and instantiate publisher for combined cloud
// and subscriber for pointcloud from wrist camera sensor
// ********************************************************************************************

// PERCEPTION CLASS CONSTRUCTOR
Perception::Perception(ros::NodeHandle nodeHandle)
{
  nodeNamespace = nodeHandle.getNamespace();
  gpdTopic = nodeNamespace + "/combined_cloud";
  combined_cloud_pub = nodeHandle.advertise<sensor_msgs::PointCloud2>(gpdTopic, 1);
  pointcloud_found = false;

  transform_listener_ptr = TransformListenerPtr(
        new tf::TransformListener());
}

// INITIALIZE SUBSCRIBER FUNCTION
// Seperate constructor for initialization of subscriber due to passing shared pointers as arguments before creating them
void Perception::init_subscriber(ros::NodeHandle nodeHandle, string camera_name)
{
  string camera_topic = "/filtered_points";
  //string camera_topic = "/" + camera_name + "/depth_registered/points";
  camera_sub = nodeHandle.subscribe(camera_topic, 1, &Perception::camera_callback, this);
}

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Camera Callback Function
// convert pointclouds from sensor_msgs::PointCloud2 to pcl::PointXYZRGB
// transform pointclouds into world frame for use
// store pointcloud in cloud list for concatenation
// shutdown the subscriber to stop further callbacks and allow a new subscriber to be built
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

// GENERIC CAMERA CALLBACK FUNCTION
void Perception::camera_callback(const sensor_msgs::PointCloud2 msg)
{
  ROS_INFO("pointcloud calback");
  PointCloud<PointXYZRGB> temp_cloud;
  fromROSMsg(msg, temp_cloud);

  ros::Time stamp = ros::Time(0);
  tf::StampedTransform transform;

  pcl_conversions::toPCL(stamp, current_cloud.header.stamp);

  // wait for and then apply transform
  try
  {
    transform_listener_ptr->waitForTransform("world", temp_cloud.header.frame_id, stamp, ros::Duration(10.0));
    transform_listener_ptr->lookupTransform("world", temp_cloud.header.frame_id, stamp, transform);
  } catch (tf::TransformException err)
  {
    ROS_ERROR("%s", err.what());
  }

  // transform the cloud to the world frame but use the same pointcloud object
  pcl_ros::transformPointCloud("world", temp_cloud, current_cloud, *transform_listener_ptr);

  // push this temporary cloud into the list of transformed clouds to allow for dynamic number of cameras to be subscribed to
  cloud_list.push_back(current_cloud);
  ROS_INFO("cloud captured, setting flag");

  pointcloud_found = true;

  // shut down the subscriber after message was received to allow a new subscriber to be generated and stop receiving messages
  camera_sub.shutdown();
}

// ********************************************************************************************
// Combined cloud publisher function
// Publish combined (concatenated) point cloud
// Convert combined_cloud (pcl::PointXYZRGB) to sensor_msgs::PointCloud2
//  and then publish
// ********************************************************************************************

// PUBLISH COMBINED CLOUD FUNCTION
void Perception::publish_combined_cloud()
{
  sensor_msgs::PointCloud2 cloud;
  toROSMsg(combined_cloud, cloud);

  combined_cloud_pub.publish(cloud);
}

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Perception Helper Functions
// function wrappers for cleaner code
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

// TAKE SNAPSHOT AND CONCATENATE IMAGES THEN PUBLISH
void Perception::generate_workspace_pointcloud(ros::NodeHandle nodeHandle)
{
  workstation_snapshot(nodeHandle);
  ros::Duration(0.5).sleep();

  int cloud_size = cloud_list.size();
  ROS_INFO("cloud list size: %d", cloud_size);
  concatenate_clouds(cloud_list, false);
  ros::Duration(0.5).sleep();

  publish_combined_cloud();
  ros::Duration(0.5).sleep();
}

// ********************************************************************************************
// Capture pointcloud functions
// take pointcloud snapshots; save camera pointclouds from camera messages to
//   variables for concatenation
// ********************************************************************************************

// TAKE CAMERA SNAPSHOTS FUNCTION
void Perception::workstation_snapshot(ros::NodeHandle nodeHandle) {
  cloud_list.clear();
  cloud_list.resize(0);

  int cameraCount = camera_names.size();
  for (int i = 0; i < cameraCount; ++i) {
    pointcloud_found = false;
    string camera_name = camera_names[i];
    init_subscriber(nodeHandle, camera_names[i]);
    ROS_INFO_STREAM("initialized subscriber for: " << camera_name);
    while(!pointcloud_found && ros::ok()) {
      // do nothing and wait
    }
  }
}

// ********************************************************************************************
// Cloud concatenation function
//
// concatenates the points of all the cloud members into the combined pointcloud
//    then performs downsampling (voxel_filter) and noise reduction (move_least_squares)
//    to clean up resulting published pointcloud
//
// ********************************************************************************************

// CONCATENATE CLOUDS FUNCTION
void Perception::concatenate_clouds(std::vector<PointCloud<PointXYZRGB>> cloud_snapshot_list, bool save_pcd)
{
  PointCloud<PointXYZRGB>::Ptr temp_cloud(new PointCloud<PointXYZRGB>);

  *temp_cloud = cloud_snapshot_list[0];
  ROS_INFO("1");

  int j = cloud_snapshot_list.size();
  if (j>1) {
    ROS_INFO("2");
    for (int i = 1; i < j; i++) {
        *temp_cloud+= cloud_snapshot_list[i];
    }
  }

  // Passthrough filter to limit to work area
  PassThrough<PointXYZRGB> pass_w;
  pass_w.setInputCloud (temp_cloud);
  pass_w.setFilterFieldName ("x");
  pass_w.setFilterLimits (-0.575, 0.575);
  pass_w.filter(*temp_cloud);

  PassThrough<PointXYZRGB> pass_y;
  pass_y.setInputCloud (temp_cloud);
  pass_y.setFilterFieldName ("y");
  pass_y.setFilterLimits (-0.425, 0.425);
  pass_y.filter(*temp_cloud);
  
  PassThrough<PointXYZRGB> pass_z;
  pass_z.setInputCloud (temp_cloud);
  pass_z.setFilterFieldName ("z");
  pass_z.setFilterLimits (0.8, 1.3);
  pass_z.filter(*temp_cloud);

  // segment out the table surface before saving to member
  sac_segmentation(temp_cloud);

  // save the pointcloud to disk
  // probably don't use this in its current state, would make huge files
  if (save_pcd){
    // save pointcloud file (.pcd) to a folder called pcd in your home directory
    pcl::io::savePCDFileASCII("~/pcd/concatenated_cloud_sample.pcd", *temp_cloud);
  }

  combined_cloud = *temp_cloud;
}
