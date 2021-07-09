
// ********************************************************************************************
// Author: Brian Flynn;
// Test Engineer - NERVE Center @ UMASS Lowell
// grasp_cluster_class.cpp
// ********************************************************************************************

#include "grasp_cluster_class.hpp"

// ********************************************************************************************
// Public Functions
// ********************************************************************************************

// CONSTRUCTOR
// --------------------------------
// Set initial varaibles and init subscriber(s)
Grasp_Cluster::Grasp_Cluster(ros::NodeHandle nodeHandle)
{
    // set planning flag to 0
    planning_grasp = 0;

    // define combined_cloud subscriber using namespace
    nodeNamespace = nodeHandle.getNamespace();
    cloudTopic = nodeNamespace + "/detect_grasps/clustered_grasps";
    clustered_grasps_gpd = nodeHandle.subscribe(cloudTopic, 1, &Grasp_Cluster::clusteredGraspsGPDCallback, this);
}

// ********************************************************************************************
// Private Functions
// ********************************************************************************************

// When clustered_grasps message is received;
// if program is not already processing list of grasps
// populate class->candidates with the incoming message,
void Grasp_Cluster::clusteredGraspsGPDCallback(gpd_ros::GraspConfigList msg)
{
  if (!planning_grasp)
  {
      numGrasps = msg.grasps.size();
      ROS_INFO("this cluster contains: %f grasps ...", numGrasps);
      if (msg.grasps.size() > 0)
      {
          candidates = msg;
          score = candidates.grasps[0].score;
          ROS_INFO("highest grasp score: %f ...", score.data);
          planning_grasp = true;
      } else {
          ROS_INFO("retrying ...");
      }
  }
}

// ********************************************************************************************
// Public Functions
// ********************************************************************************************

// Return current grasp candidates for processing in manipulation function
gpd_ros::GraspConfigList Grasp_Cluster::get_grasp_candidates()
{
    return candidates;
}

// Set planning_grasp variable 1 or 0 for callback function
// take in 0 or 1 to turn further grasp candidate saving on or off
void Grasp_Cluster::set_planning(bool val)
{
    planning_grasp = val;
}

void Grasp_Cluster::set_retry(bool val)
{
    retry = val;
}
