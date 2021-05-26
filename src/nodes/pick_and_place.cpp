// ********************************************************************************************
// Author: Brian Flynn;
// NERVE Center @ UMASS Lowell
// pick_and_place_s_cams_gpd.cpp
//
// perform a pick and place operation using the stationary left, right and back cameras
// on the workstation and gpd to plan grasp poses
// ********************************************************************************************

#include "manipulation_class.hpp"
#include "perception_class.hpp"
#include "grasp_cluster_class.hpp"

int main(int argc, char** argv)
{
  // Initialize node & spinner
  ros::init(argc,argv,"pick_and_place_node");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(4);
  spinner.start();
  ros::Duration(1.0).sleep();

  // Instantiate and retrieve planning group from parameters, this should be set outside the node
  string planning_group;
  if (nh.getParam("/planning_group", planning_group))
  {
    ROS_INFO("Planning group is: %s ...", planning_group.c_str());
  } else
  {
    ROS_INFO("Failed to retrieve planning group parameter ...");
  }

  // Create manipulation, perception and grasping objects
  Manipulation manipulation(nh, planning_group);
  Perception perception(nh);
  Grasp_Cluster grasp_cluster(nh);

  // Set planning variables and reset robot position
  manipulation.move_group_ptr->setPlanningTime(15.0);basic_manipulation_and_3d_perception_node
  manipulation.move_group_ptr->setMaxVelocityScalingFactor(0.25);
  manipulation.move_group_ptr->setPlannerId("RRTConnect");

  //manipulation.move_group_ptr->setNamedTarget("preset_1");
  manipulation.move_group_ptr->setNamedTarget("retract");
  manipulation.move_group_ptr->move();

  while(ros::ok())
  {
      // take snapshot and publish resulting concatenated pointcloud
      perception.generate_workspace_pointcloud();
      //perception.generate_wrist_pointcloud();

      while (!grasp_cluster.planning_grasp) {
          if (grasp_cluster.retry) {
              perception.generate_workspace_pointcloud();
              grasp_cluster.set_retry(0);
          }
      }

      // this function needs the namespace (pass it in here for subscribed topic)
      manipulation.store_gpd_vals(grasp_cluster.get_grasp_candidates());
      manipulation.createPickingEEFPoseList();

      //manipulation.pick_and_place(manipulation.graspPoseList, manipulation.placePose);
      manipulation.pick(manipulation.graspPoseList);
      //gripper.setGripper();

      ros::Duration(0.5).sleep();

      // move back to a neutral position
      manipulation.move_group_ptr->setNamedTarget("retract");
      //manipulation.move_group_ptr->setNamedTarget("preset_1");
      manipulation.move_group_ptr->move();

      // set planning flag to OK for next loop
      grasp_cluster.set_planning(0);
      ros::Duration(0.5).sleep();
  }

  ros::waitForShutdown();
  return 0;

}
