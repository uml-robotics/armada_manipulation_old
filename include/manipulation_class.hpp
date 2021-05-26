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
};

class Manipulation
{
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
    std::vector<GraspPose> graspPoseList;
    gpd::GraspConfigList candidates;

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
    void setGripper(trajectory_msgs::JointTrajectory& posture, double closeVal);

    //Functions
    void pick(std::vector<GraspPose> graspPoseList);
    void place(geometry_msgs::Pose placePose);
    void pick_and_place(std::vector<GraspPose> graspPoseList, geometry_msgs::Pose placePose);
    void set_target_pose_rpy(geometry_msgs::Pose graspPose);
    void set_target_pose_from_grasps(geometry_msgs::Pose graspPositionPose, geometry_msgs::Pose graspOrientationPose);
    bool plan(geometry_msgs::Pose graspPositionPose, geometry_msgs::Pose graspOrientationPose);
    void executeGrasp(GraspPose graspPose);
    void plan_and_move();

    //Helper Funcitons
    void store_gpd_vals(gpd::GraspConfigList candidates);
    void createPickingEEFPoseList();
    GraspPose createPickingEEFPose(gpd::GraspConfig grasp_msg);

};

#endif // MANIPULATION_CLASS
