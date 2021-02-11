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
  this->combined_cloud_pub = nodeHandle.advertise<sensor_msgs::PointCloud2>("combined_cloud", 1);
  this->points_not_found = true;
}

// INITIALIZE SUBSCRIBER FUNCTION
// Seperate constructor for initialization of subscriber due to passing shared pointers as arguments before creating them
void Perception::init_subscriber(ros::NodeHandle nodeHandle)
{
  this->left_camera_sub = nodeHandle.subscribe("/camera_left/depth/points", 1, &Perception::left_camera_callback, this);
  this->right_camera_sub = nodeHandle.subscribe("/camera_right/depth/points", 1, &Perception::right_camera_callback, this);
  this->rear_camera_sub = nodeHandle.subscribe("/camera_rear/depth/points", 1, &Perception::rear_camera_callback, this);
}

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Camera Callback Functions
// convert pointclouds from sensor_msgs::PointCloud2 to pcl::PointXYZRGB
// transform pointclouds into world frame for use
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

// LEFT CAMERA CALLBACK FUNCTION
void Perception::left_camera_callback(const sensor_msgs::PointCloud2 msg)
{
    fromROSMsg(msg, this->left_cloud);

    ros::Time stamp = ros::Time(0);
    pcl_conversions::toPCL(stamp, this->left_cloud.header.stamp);
    this->transform_listener_ptr->waitForTransform("/world", this->left_cloud.header.frame_id, stamp, ros::Duration(10.0));
    pcl_ros::transformPointCloud("/world", this->left_cloud, this->left_cloud, *this->transform_listener_ptr);
}

// RIGHT CAMERA CALLBACK FUNCTION
void Perception::right_camera_callback(const sensor_msgs::PointCloud2 msg)
{
    fromROSMsg(msg, this->right_cloud);

    ros::Time stamp = ros::Time(0);
    pcl_conversions::toPCL(stamp, this->right_cloud.header.stamp);
    this->transform_listener_ptr->waitForTransform("/world", this->right_cloud.header.frame_id, stamp, ros::Duration(10.0));
    pcl_ros::transformPointCloud("/world", this->right_cloud, this->right_cloud, *this->transform_listener_ptr);
}

// REAR CAMERA CALLBACK FUNCTION
void Perception::rear_camera_callback(const sensor_msgs::PointCloud2 msg)
{
    fromROSMsg(msg, this->rear_cloud);

    ros::Time stamp = ros::Time(0);
    pcl_conversions::toPCL(stamp, this->rear_cloud.header.stamp);
    this->transform_listener_ptr->waitForTransform("/world", this->rear_cloud.header.frame_id, stamp, ros::Duration(10.0));
    pcl_ros::transformPointCloud("/world", this->rear_cloud, this->rear_cloud, *this->transform_listener_ptr);
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
  toROSMsg(this->combined_cloud, cloud);

  this->combined_cloud_pub.publish(cloud);
}

// ********************************************************************************************
// Capture pointcloud functions
// take pointcloud snapshots; save camera pointclouds from camera messages to
//  variables for concatenation
// ********************************************************************************************

// TAKE CAMERA SNAPSHOTS FUNCTION
void Perception::collect_camera_snapshots() {
  snapshot_left_pointcloud();
  snapshot_rear_pointcloud();
  snapshot_right_pointcloud();
}

// SNAPSHOT LEFT POINTCLOUD FUNCTION
void Perception::snapshot_left_pointcloud()
{
  this->left_cloud_snapshot = this->left_cloud;
}

// SNAPSHOT RIGHT POINTCLOUD FUNCTION
void Perception::snapshot_right_pointcloud()
{
  this->right_cloud_snapshot = this->right_cloud;
}

// SNAPSHOT REAR POINTCLOUD FUNCTION
void Perception::snapshot_rear_pointcloud()
{
  this->rear_cloud_snapshot = this->rear_cloud;
}

// ********************************************************************************************
// Cloud concatenation function
// concatenates the points of all the cloud members into the combined pointcloud
//  then performs downsampling (voxel_filter) and noise reduction (move_least_squares)
//  to clean up resulting published pointcloud
// ********************************************************************************************

// CONCATENATE CLOUDS FUNCTION
void Perception::concatenate_clouds() 
{
  PointCloud<PointXYZRGB>::Ptr temp_cloud(new PointCloud<PointXYZRGB>);
    
  *temp_cloud = this->left_cloud_snapshot;
  *temp_cloud+= this->right_cloud_snapshot;
  *temp_cloud+= this->rear_cloud_snapshot;

  // Uncomment to save concatenated pointcloud if desired
  // pcl::io::savePCDFileASCII("~/pcd/single_workstation_object_sample.pcd", *temp_cloud);

  /*
  // Passthrough filter to limit to work area
  PassThrough<PointXYZRGB> pass_w;
  pass_w.setInputCloud (temp_cloud);
  pass_w.setFilterFieldName ("x");
  pass_w.setFilterLimits (-0.9, -0.3);
  pass_w.filter(*temp_cloud);

  PassThrough<PointXYZRGB> pass_y;
  pass_y.setInputCloud (temp_cloud);
  pass_y.setFilterFieldName ("y");
  pass_y.setFilterLimits (-0.45, 0.45);
  pass_y.filter(*temp_cloud);
  
  PassThrough<PointXYZRGB> pass_z;
  pass_z.setInputCloud (temp_cloud);
  pass_z.setFilterFieldName ("z");
  pass_z.setFilterLimits (-0.02, 0.3);
  pass_z.filter(*temp_cloud);
  */

  this->combined_cloud = *temp_cloud;
}

// ********************************************************************************************
// Generic PCL Filters
// ********************************************************************************************

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
