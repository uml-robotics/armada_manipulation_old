// ********************************************************************************************
// Author: Brian Flynn;
// Test Engineer - NERVE Center @ UMASS Lowell
// perception_class.cpp
//
// perception_class function definitions
// ********************************************************************************************

#include "perception_class.hpp"

// PERCEPTION CLASS CONSTRUCTOR
Perception::Perception(ros::NodeHandle nodeHandle)
{
  string nodeNamespace = nodeHandle.getNamespace();
  string gpdTopic = nodeNamespace + "/combined_cloud";
  combined_cloud_pub = nodeHandle.advertise<sensor_msgs::PointCloud2>(gpdTopic, 1);

  transform_listener_ptr = TransformListenerPtr(
        new tf::TransformListener());

  // Retrieve list of camera names from ros parameter
  cloud_list.resize(0);
  camera_names.resize(0);
  nodeHandle.getParam("/camera_names", camera_names);
  camera_count = camera_names.size();
}

// INITIALIZE SUBSCRIBER FUNCTION
// Seperate constructor for initialization of subscriber due to passing shared pointers as arguments before creating them
void Perception::initSubscriber(ros::NodeHandle nodeHandle, string camera_name)
{
  string camera_topic = "/" + camera_name + "/depth/points";
  camera_sub = nodeHandle.subscribe(camera_topic, 1, &Perception::cameraCallback, this);
  cloud_stored = false;
}

// GENERIC CAMERA CALLBACK FUNCTION
void Perception::cameraCallback(const sensor_msgs::PointCloud2 msg)
{
  if (!cloud_stored){
    current_cloud = msg;
    cloud_stored = true;
  }
}

// PUBLISH COMBINED CLOUD FUNCTION
void Perception::publishCombinedCloud(PointCloud<PointXYZRGB> input_cloud)
{
  sensor_msgs::PointCloud2 cloud;
  toROSMsg(input_cloud, cloud);

  combined_cloud_pub.publish(cloud);
}

// MULTI CAMERA SNAPSHOTS FUNCTION
void Perception::multiCameraSnapshot(ros::NodeHandle nodeHandle)
{
  cloud_list.resize(0);

  for (int i = 0; i < camera_count; ++i) {
    initSubscriber(nodeHandle, camera_names[i].c_str());
    ros::Duration(1.0).sleep();
    cloud_list.push_back(transformCloud(current_cloud));
  }
}

// WRIST CAMERA SNAPSHOT FUNCTION
void Perception::wristCameraSnapshot(ros::NodeHandle nodeHandle, string camera_name)
{
  initSubscriber(nodeHandle, camera_name);
  ros::Duration(1.0).sleep();
  cloud_list.push_back(transformCloud(current_cloud));
}

// TRANSFORM POINTCLOUD FUNCTION
PointCloud<PointXYZRGB> Perception::transformCloud(sensor_msgs::PointCloud2 cloud){
  PointCloud<PointXYZRGB> temp_cloud;
  fromROSMsg(cloud, temp_cloud);

  ros::Time stamp = ros::Time(0);
  tf::StampedTransform transform;

  pcl_conversions::toPCL(stamp, temp_cloud.header.stamp);

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
  pcl_ros::transformPointCloud("world", temp_cloud, temp_cloud, *transform_listener_ptr);

  return temp_cloud;
}

// CONCATENATE CLOUDS FUNCTION
PointCloud<PointXYZRGB> Perception::concatenateClouds(std::vector<PointCloud<PointXYZRGB>> cloud_snapshot_list)
{
  PointCloud<PointXYZRGB>::Ptr temp_cloud(new PointCloud<PointXYZRGB>);

  *temp_cloud = cloud_snapshot_list[0];

  int j = cloud_snapshot_list.size();
  if (j>1) {
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

  sac_segmentation(temp_cloud);

  return *temp_cloud;
}

void savePointCloudToDisk(PointCloud<PointXYZRGB> cloud, string filepath)
{
  PointCloud<PointXYZRGB>::Ptr temp_cloud(new PointCloud<PointXYZRGB>);
  *temp_cloud = cloud;

  // ex. filepath = "~/pcd/concatenated_cloud_sample.pcd"
  pcl::io::savePCDFileASCII(filepath, *temp_cloud);
}
