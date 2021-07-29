#include "ros/ros.h"
#include "uml_hri_nerve_pick_and_place/CapturePointCloud.h"

bool add(uml_hri_nerve_pick_and_place::CapturePointCloud::Request  &req,
         uml_hri_nerve_pick_and_place::CapturePointCloud::Response &res)
{
  ROS_INFO("test");
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "capture_pointcloud_service");
  ros::NodeHandle nh;

  ros::ServiceServer service = nh.advertiseService("capture_pointcloud", add);
  ROS_INFO("Ready to capture a pointcloud.");
  ros::spin();

  return 0;
}
