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
  points_not_found = true;
  init_subscriber(nodeHandle);

  transform_listener_ptr = TransformListenerPtr(
        new tf::TransformListener());
}

// INITIALIZE SUBSCRIBER FUNCTION
// Seperate constructor for initialization of subscriber due to passing shared pointers as arguments before creating them
void Perception::init_subscriber(ros::NodeHandle nodeHandle)
{
  base_camera_sub = nodeHandle.subscribe("/camera_base/depth/points", 1, &Perception::base_camera_callback, this);
  wrist_camera_sub = nodeHandle.subscribe("/camera/depth/points", 1, &Perception::wrist_camera_callback, this);
  left_camera_sub = nodeHandle.subscribe("/camera_left/depth/points", 1, &Perception::left_camera_callback, this);
  right_camera_sub = nodeHandle.subscribe("/camera_right/depth/points", 1, &Perception::right_camera_callback, this);
  rear_camera_sub = nodeHandle.subscribe("/camera_rear/depth/points", 1, &Perception::rear_camera_callback, this);
}

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Camera Callback Functions
// convert pointclouds from sensor_msgs::PointCloud2 to pcl::PointXYZRGB
// transform pointclouds into world frame for use
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

// BASE CAMERA CALLBACK FUNCTION
void Perception::base_camera_callback(const sensor_msgs::PointCloud2 msg)
{
    PointCloud<PointXYZRGB> temp_base_cloud;
    fromROSMsg(msg, temp_base_cloud);

    ros::Time stamp = ros::Time(0);
    tf::StampedTransform transform;

    pcl_conversions::toPCL(stamp, base_cloud.header.stamp);

    try
    {
        transform_listener_ptr->waitForTransform("world", temp_base_cloud.header.frame_id, stamp, ros::Duration(10.0));
        transform_listener_ptr->lookupTransform("world", temp_base_cloud.header.frame_id, stamp, transform);
    } catch (tf::TransformException err)
    {
        ROS_ERROR("%s", err.what());
    }

    pcl_ros::transformPointCloud("world", temp_base_cloud, base_cloud, *transform_listener_ptr);
}

// WRIST CAMERA CALLBACK FUNCTION
void Perception::wrist_camera_callback(const sensor_msgs::PointCloud2 msg)
{
    PointCloud<PointXYZRGB> temp_wrist_cloud;
    fromROSMsg(msg, temp_wrist_cloud);

    ros::Time stamp = ros::Time(0);
    tf::StampedTransform transform;

    pcl_conversions::toPCL(stamp, wrist_cloud.header.stamp);

    try
    {
        transform_listener_ptr->waitForTransform("world", temp_wrist_cloud.header.frame_id, stamp, ros::Duration(10.0));
        transform_listener_ptr->lookupTransform("world", temp_wrist_cloud.header.frame_id, stamp, transform);
    } catch (tf::TransformException err)
    {
        ROS_ERROR("%s", err.what());
    }

    pcl_ros::transformPointCloud("world", temp_wrist_cloud, wrist_cloud, *transform_listener_ptr);
}

// LEFT CAMERA CALLBACK FUNCTION
void Perception::left_camera_callback(const sensor_msgs::PointCloud2 msg)
{
    PointCloud<PointXYZRGB> temp_left_cloud;
    fromROSMsg(msg, temp_left_cloud);

    ros::Time stamp = ros::Time(0);
    tf::StampedTransform transform;

    pcl_conversions::toPCL(stamp, left_cloud.header.stamp);

    try
    {
        transform_listener_ptr->waitForTransform("world", temp_left_cloud.header.frame_id, stamp, ros::Duration(10.0));
        transform_listener_ptr->lookupTransform("world", temp_left_cloud.header.frame_id, stamp, transform);
    } catch (tf::TransformException err)
    {
        ROS_ERROR("%s", err.what());
    }

    pcl_ros::transformPointCloud("world", temp_left_cloud, left_cloud, *transform_listener_ptr);
}

// RIGHT CAMERA CALLBACK FUNCTION
void Perception::right_camera_callback(const sensor_msgs::PointCloud2 msg)
{
    PointCloud<PointXYZRGB> temp_right_cloud;
    fromROSMsg(msg, temp_right_cloud);

    ros::Time stamp = ros::Time(0);
    tf::StampedTransform transform;

    pcl_conversions::toPCL(stamp, temp_right_cloud.header.stamp);

    try
    {
        transform_listener_ptr->waitForTransform("world", temp_right_cloud.header.frame_id, stamp, ros::Duration(10.0));
        transform_listener_ptr->lookupTransform("world", temp_right_cloud.header.frame_id, stamp, transform);
    } catch (tf::TransformException err)
    {
        ROS_ERROR("%s", err.what());
    }

    pcl_ros::transformPointCloud("world", temp_right_cloud, right_cloud, *transform_listener_ptr);
}

// REAR CAMERA CALLBACK FUNCTION
void Perception::rear_camera_callback(const sensor_msgs::PointCloud2 msg)
{
    PointCloud<PointXYZRGB> temp_rear_cloud;
    fromROSMsg(msg, temp_rear_cloud);

    ros::Time stamp = ros::Time(0);
    tf::StampedTransform transform;

    pcl_conversions::toPCL(stamp, temp_rear_cloud.header.stamp);

    try
    {
        transform_listener_ptr->waitForTransform("world", temp_rear_cloud.header.frame_id, stamp, ros::Duration(10.0));
        transform_listener_ptr->lookupTransform("world", temp_rear_cloud.header.frame_id, stamp, transform);
    } catch (tf::TransformException err)
    {
        ROS_ERROR("%s", err.what());
    }

    pcl_ros::transformPointCloud("world", temp_rear_cloud, rear_cloud, *transform_listener_ptr);
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
//
// these functions will be replaced with one function that calls an action
//    server which returns just the pointclouds that are relevant to
//    whatever the launch file/user specifies for the setup (will not
//    be implementing this in this push due to time constraints)
//
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

// TAKE SNAPSHOT AND CONCATENATE IMAGES THEN PUBLISH
void Perception::generate_workspace_pointcloud()
{
    concatenate_clouds(workstation_snapshot());
    ros::Duration(0.5).sleep();

    publish_combined_cloud();
    ros::Duration(0.5).sleep();
}

// TAKE SNAPSHOT AND CONCATENATE IMAGES THEN PUBLISH
void Perception::generate_base_cam_pointcloud()
{
    concatenate_clouds(base_cam_snapshot());
    ros::Duration(0.5).sleep();

    publish_combined_cloud();
    ros::Duration(0.5).sleep();
}

// TAKE SNAPSHOT OF ONLY WRIST CAMERA AND PUBLISH
void Perception::generate_wrist_pointcloud()
{
    concatenate_clouds(wrist_cam_snapshots());
    ros::Duration(0.5).sleep();

    publish_combined_cloud();
    ros::Duration(0.5).sleep();
}

// ********************************************************************************************
// Capture pointcloud functions
// take pointcloud snapshots; save camera pointclouds from camera messages to
//  variables for concatenation
// ********************************************************************************************

// TAKE CAMERA SNAPSHOTS FUNCTION
std::vector<PointCloud<PointXYZRGB>> Perception::workstation_snapshot() {
  cloud_list.clear();
  cloud_list.resize(0);
  cloud_list.push_back(left_cloud);
  cloud_list.push_back(right_cloud);
  cloud_list.push_back(rear_cloud);

  return cloud_list;
}

// SNAPSHOT WRIST POINTCLOUD FUNCTION
// This function should be called after moving the robot to a position in the main node
std::vector<PointCloud<PointXYZRGB>> Perception::wrist_cam_snapshots()
{
  cloud_list.clear();
  cloud_list.resize(0);
  cloud_list.push_back(wrist_cloud);

  return cloud_list;
}

// SNAPSHOT WRIST POINTCLOUD FUNCTION
std::vector<PointCloud<PointXYZRGB>> Perception::base_cam_snapshot()
{
  cloud_list.clear();
  cloud_list.resize(0);
  cloud_list.push_back(base_cloud);

  return cloud_list;
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
void Perception::concatenate_clouds(std::vector<PointCloud<PointXYZRGB>> cloud_snapshot_list)
{
  PointCloud<PointXYZRGB>::Ptr temp_cloud(new PointCloud<PointXYZRGB>);

  *temp_cloud = cloud_snapshot_list[0];

  int j = cloud_snapshot_list.size();
  if (j>1) {
    for (int i = 1; i < j; i++) {
        *temp_cloud+= cloud_snapshot_list[i];
    }
  }

  // Uncomment to save concatenated pointcloud if desired
  // pcl::io::savePCDFileASCII("~/pcd/single_workstation_object_sample.pcd", *temp_cloud);

  // Passthrough filter to limit to work area
  PassThrough<PointXYZRGB> pass_w;
  pass_w.setInputCloud (temp_cloud);
  pass_w.setFilterFieldName ("x");
  pass_w.setFilterLimits (-0.6, 0.6);
  pass_w.filter(*temp_cloud);

  PassThrough<PointXYZRGB> pass_y;
  pass_y.setInputCloud (temp_cloud);
  pass_y.setFilterFieldName ("y");
  pass_y.setFilterLimits (-0.45, 0.45);
  pass_y.filter(*temp_cloud);
  
  PassThrough<PointXYZRGB> pass_z;
  pass_z.setInputCloud (temp_cloud);
  pass_z.setFilterFieldName ("z");
  pass_z.setFilterLimits (0.8, 1.2);
  pass_z.filter(*temp_cloud);

  this->combined_cloud = *temp_cloud;
}

// ****************************************************************************************************************************************************************************************
// Generic PCL Filters
// ****************************************************************************************************************************************************************************************

// COMPUTE NORMALS FUNCTION
// TODO: add description
void Perception::computeNormals(PointCloud<PointXYZRGB>::Ptr cloud, PointCloud<Normal>::Ptr cloud_normals)
{
  search::KdTree<PointXYZRGB>::Ptr tree(new search::KdTree<PointXYZRGB>());
  NormalEstimation<PointXYZRGB, Normal> ne;
  ne.setSearchMethod(tree);
  ne.setInputCloud(cloud);
  ne.setKSearch(50);
  ne.compute(*cloud_normals);
}

// EXTRACT NORMALS FUNCTION
// TODO: add description
void Perception::extractNormals(PointCloud<Normal>::Ptr cloud_normals, PointIndices::Ptr inliers_plane)
{
  ExtractIndices<Normal> extract_normals;
  extract_normals.setNegative(true);
  extract_normals.setInputCloud(cloud_normals);
  extract_normals.setIndices(inliers_plane);
  extract_normals.filter(*cloud_normals);
}

// VOXEL FILTER FUNCTION
// downsamples point cloud to make the resulting model cleaner
PointCloud<PointXYZRGB> Perception::voxelgrid_filter(PointCloud<PointXYZRGB>::Ptr cloud)
{
  PointCloud<PointXYZRGB>::Ptr filtered_cloud(new PointCloud<PointXYZRGB>);

  VoxelGrid<PointXYZRGB> sor;
  sor.setInputCloud(cloud);
  sor.setLeafSize(0.010, 0.010, 0.010);
  sor.filter(*filtered_cloud);

  return *filtered_cloud;
}

// SAC (PLANAR) SEGMENTATION FUNCTION
// remove largest planar surface from pointcloud
PointCloud<PointXYZRGB> Perception::sac_segmentation(PointCloud<PointXYZRGB>::Ptr cloud, PointIndices::Ptr inliers_plane)
{
  // Create the segmentation object
  SACSegmentation<PointXYZRGB> seg;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (SACMODEL_PLANE);
  seg.setMethodType (SAC_RANSAC);
  seg.setMaxIterations (1000);
  seg.setDistanceThreshold (0.01);
  seg.setInputCloud (cloud);
  ModelCoefficients::Ptr coefficients_plane (new ModelCoefficients);
  seg.segment (*inliers_plane, *coefficients_plane);

  // Create the filtering object
  ExtractIndices<PointXYZRGB> extract_indices;
  extract_indices.setInputCloud(cloud);
  extract_indices.setIndices(inliers_plane);
  extract_indices.setNegative(true);
  extract_indices.filter(*cloud);

  return *cloud;
}

// MOVE LEAST SQUARES FUNCTION
// aligns the surface normals to eliminate noise
PointCloud<PointNormal> Perception::move_least_squares(PointCloud<PointXYZRGB>::Ptr cloud)
{
  search::KdTree<PointXYZRGB>::Ptr tree(new search::KdTree<PointXYZRGB>);
  PointCloud<PointNormal> mls_points;
  MovingLeastSquares<PointXYZRGB, PointNormal> mls;
 
  mls.setComputeNormals(true);
  mls.setInputCloud(cloud);
  mls.setPolynomialOrder(4);
  mls.setSearchMethod(tree);
  mls.setSearchRadius(0.1);
  mls.process(mls_points);

  return mls_points;
}
