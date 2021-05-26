// ********************************************************************************************
// Author: Brian Flynn;
// Test Engineer - NERVE Center @ UMASS Lowell
// grasp_cluster_class.hpp
// ********************************************************************************************

#ifndef GRASP_CLUSTER_CLASS_HPP
#define GRASP_CLUSTER_CLASS_HPP

#include <cstring>
#include <ctime>
#include <iostream>

#include "ros/ros.h"
#include <gpd/GraspConfigList.h>
#include <gpd/GraspConfig.h>

#include <array>
#include <std_srvs/Empty.h>

#include <boost/filesystem.hpp>

#include <geometry_msgs/Pose.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace std;

class Grasp_Cluster
{
  private:

    // Subscriber
    ros::Subscriber clustered_grasps_gpd;

    // Functions
    void clusteredGraspsGPDCallback(const gpd::GraspConfigList msg);

  public:

    //Node Member Variables
    string nodeNamespace;
    string cloudTopic;

    // GPD Grasp members
    gpd::GraspConfigList candidates;
    gpd::GraspConfig grasp;
    std_msgs::Float32 score;
    double numGrasps;

    // Constructor
    Grasp_Cluster(ros::NodeHandle nodeHandle);

    // Functions
    gpd::GraspConfigList get_grasp_candidates();
    void set_planning(bool val);
    void set_retry(bool val);

    // Flags and triggers
    bool planning_grasp;
    bool retry;

};

#endif // GRASP_CLUSTER_CLASS
