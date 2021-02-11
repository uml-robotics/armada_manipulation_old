
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
  this->planning_grasp = 0;
  this->clustered_grasps_gpd = nodeHandle.subscribe("/detect_grasps/clustered_grasps", 1, &Grasp_Cluster::clusteredGraspsGPDCallback, this);
}

// get_grasp_candidates() function
// --------------------------------
// Return current grasp candidates for processing in manipulation function
gpd::GraspConfigList Grasp_Cluster::get_grasp_candidates()
{
  ROS_INFO("getting candidates");
  ROS_INFO("new cluster grabbed, highest score: %f", this->score.data);
  return this->candidates;
}

// set_planning() function
// --------------------------------
// Set planning_grasp variable 1 or 0 for callback function
// take in 0 or 1 to turn further grasp candidate saving on or off
void Grasp_Cluster::set_planning(bool val)
{
  this->planning_grasp = val;
}

// ********************************************************************************************
// Private Functions
// ********************************************************************************************

// clustered_grasps_gpd subscriber callback function
// --------------------------------
// When clustered_grasps message is received;
// if program is not already processing list of grasps
// populate class->candidates with the incoming message,
// otherwise do nothing
void Grasp_Cluster::clusteredGraspsGPDCallback(gpd::GraspConfigList msg)
{
  if (msg.grasps.size() > 0)
  {
    if (this->planning_grasp == 0)
    {
      this->candidates = msg;
      this->score = candidates.grasps[0].score;
      if (this->score.data > -150)
      {
        ROS_INFO("new cluster grabbed, highest score: %f", this->score.data);
        this->planning_grasp = 1;
      }
      else
      {
        ROS_INFO("trying new cluster, highest score was: %f", this->score.data);
      }
    }
  }
  if (msg.grasps.size() == 0)
  {
    ROS_INFO("zero grasps, retry");
  }
}
