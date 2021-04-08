// ********************************************************************************************
// Author: Brian Flynn;
// Test Engineer - NERVE Center @ UMASS Lowell
// manipulation_class.hpp
// ********************************************************************************************

#ifndef MANIPULATION_CLASS_HPP
#define MANIPULATION_CLASS_HPP

#include <boost/filesystem.hpp>
#include <math.h>
#include <stdlib.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <array>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <moveit/move_group/capability_names.h>
#include <std_srvs/Empty.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_listener.h>
#include <std_msgs/Int8.h>
#include <gpd/GraspConfigList.h>
#include <gpd/GraspConfig.h>

typedef boost::shared_ptr<moveit::planning_interface::MoveGroupInterface> MoveGroupPtr;
typedef boost::shared_ptr<moveit::planning_interface::PlanningSceneInterface> PlanningScenePtr;
typedef boost::shared_ptr<tf::TransformListener> TransformListenerPtr;

struct GraspPose{
  geometry_msgs::Pose pre;
  geometry_msgs::Pose actual;
  geometry_msgs::Pose after;
  float distance;
};

class Manipulation
{
  private:

    //Publishers & Subscribers
    ros::Publisher placeholder_pub;
    ros::Subscriber placeholder_sub;

    //Subscriber Callbacks
    void placeholder_sub_callback(const int num);

    //Functions

  public:

    //Constructor
    Manipulation(ros::NodeHandle nodeHandle, std::string planning_group);

    //Ros Service Member Variables
    ros::ServiceClient clearOctomap;
    std_srvs::Empty srv;

    //Moveit! Member Variables
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    moveit::core::RobotStatePtr current_state;

    //Grasp Planning Member Variables
    GraspPose grasp_poses;
    gpd::GraspConfigList candidates;
    gpd::GraspConfig grasp;
    geometry_msgs::Point pose_top;
    geometry_msgs::Point pose_bottom;
    geometry_msgs::Point pose_center;
    geometry_msgs::Point pose_sample;
    geometry_msgs::Vector3 grasp_orientation;

    //Manipulation Pose Member Variables
    std::vector<double> joint_group_positions;
    geometry_msgs::Pose target_pose;
    geometry_msgs::Vector3 orientation;
    tf2::Quaternion q;

    //Pointer Variables
    MoveGroupPtr move_group_ptr;
    PlanningScenePtr planning_scene_ptr;
    TransformListenerPtr transform_listener_ptr;

    //Flag Variables
    bool pose_success;

    //Gripper Functions
    void openGripper(trajectory_msgs::JointTrajectory& posture);
    void closeGripper(trajectory_msgs::JointTrajectory& posture);
    void closeVarGripper(trajectory_msgs::JointTrajectory& posture, double closeVal);

    //Functions
    void pick(GraspPose graspPoses);
    void place(GraspPose graspPoses);
    void pick_and_place();
    void pick_and_place(GraspPose graspPose);
    void set_target_pose(geometry_msgs::Pose graspPose);
    void go_to_poses_test(geometry_msgs::Pose graspPose);


    //Helper Funcitons
    void store_gpd_vals(gpd::GraspConfigList candidates);
    void createPickingEEFPose(gpd::GraspConfig grasp_msg);
    void addCollisionObjects(GraspPose graspPoses);

};

#endif // MANIPULATION_CLASS
