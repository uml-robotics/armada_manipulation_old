// ********************************************************************************************
// Author: Brian Flynn;
// Test Engineer - NERVE Center @ UMASS Lowell
// perception_class.hpp
//
// Defines a Perception class to handle visualization for operating a robotic manipulator
// utilizing an Intel Realsense D435i sensor 
//
// ********************************************************************************************

#ifndef PERCEPTION_CLASS_HPP
#define PERCEPTION_CLASS_HPP

#include <boost/filesystem.hpp>
#include <cstring>
#include <ctime>
#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>				// saving pointclouds
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/obj_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/mls.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf/transform_listener.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>

typedef boost::shared_ptr<tf::TransformListener> TransformListenerPtr;

using namespace pcl;
using namespace std;
using namespace tf;

class Perception
{
  private:
  
    //Publishers & Subscribers
    ros::Publisher combined_cloud_pub;
    ros::Subscriber camera_sub;

    //Subscriber Callbacks
    void cameraCallback(const sensor_msgs::PointCloud2 msg);

    //Listener Pointers
    TransformListenerPtr transform_listener_ptr;

    //Generic PCL Filters
    PointCloud<PointXYZRGB> voxelgrid_filter(PointCloud<PointXYZRGB>::Ptr cloud);
    PointCloud<PointXYZRGB> sac_segmentation(PointCloud<PointXYZRGB>::Ptr cloud);
    PointCloud<PointNormal> move_least_squares(PointCloud<PointXYZRGB>::Ptr cloud);
    void computeNormals(PointCloud<PointXYZRGB>::Ptr cloud, PointCloud<Normal>::Ptr cloud_normals);
    void extractNormals(PointCloud<Normal>::Ptr cloud_normals, PointIndices::Ptr inliers_plane);

  public:

    //Constructor
    Perception(ros::NodeHandle nodeHandle);

    //Generic Subscriber initializer
    void initSubscriber(ros::NodeHandle nodeHandle, string camera_name);

    //Node Member Variables

    // Topic Name Member Variables
    std::vector<string> camera_names;
    int camera_count;

    //Pointcloud Member Variables
    sensor_msgs::PointCloud2 current_cloud;
    std::vector<PointCloud<PointXYZRGB>> cloud_list;

    //Flag Variables
    bool cloud_stored;

    //Functions
    void publishCombinedCloud(PointCloud<PointXYZRGB> cloud);
    void multiCameraSnapshot(ros::NodeHandle nodeHandle);
    void wristCameraSnapshot(ros::NodeHandle nodeHandle, string camera_name);
    PointCloud<PointXYZRGB> transformCloud(sensor_msgs::PointCloud2 cloud);
    PointCloud<PointXYZRGB> concatenateClouds(std::vector<PointCloud<PointXYZRGB>> cloud_snapshot_list);

    //Helper Function/Function Wrappers
    void savePointCloudToDisk(PointCloud<PointXYZRGB> cloud, string filepath);
};  

#endif // PERCEPTION_CLASS
