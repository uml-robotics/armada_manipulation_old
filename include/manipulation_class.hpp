// ********************************************************************************************
// Author: Brian Flynn;
// Test Engineer - NERVE Center @ UMASS Lowell
// manipulation_class.hpp
// ********************************************************************************************

#ifndef MANIPULATION_CLASS_HPP
#define MANIPULATION_CLASS_HPP

#include <cstring>
#include <ctime>
#include <iostream>
#include <boost/filesystem.hpp>
#include <math.h>
#include <stdlib.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <control_msgs/GripperCommandActionGoal.h>
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
#include <gpd_ros/GraspConfigList.h>
#include <gpd_ros/GraspConfig.h>

using namespace std;

typedef boost::shared_ptr<moveit::planning_interface::MoveGroupInterface> MoveGroupPtr;
typedef boost::shared_ptr<moveit::planning_interface::PlanningSceneInterface> PlanningScenePtr;
typedef boost::shared_ptr<tf::TransformListener> TransformListenerPtr;

struct GraspPose{
  geometry_msgs::Pose pre;
  geometry_msgs::Pose actual;
  geometry_msgs::Pose after;
};

struct GraspPlan{
  bool preResult;
  bool actualResult;
  bool afterResult;
  moveit::planning_interface::MoveGroupInterface::Plan prePlan;
  moveit::planning_interface::MoveGroupInterface::Plan actualPlan;
  moveit::planning_interface::MoveGroupInterface::Plan afterPlan;
};

class Manipulation
{
  public:

    //Constructor
    Manipulation(ros::NodeHandle nodeHandle, std::string planning_group);
    void getParams(ros::NodeHandle nh);

    //Grasping Member Variables
    control_msgs::GripperCommandActionGoal gripper_cmd;
    ros::Publisher gripper_command;

    //Pointer Variables
    MoveGroupPtr move_group_ptr;
    PlanningScenePtr planning_scene_ptr;
    TransformListenerPtr transform_listener_ptr;

    //move_group Member Variables
    double MaxVelocityScalingFactor;
    double PlanningTime;
    int NumPlanningAttempts;
    double GoalPositionTolerance;
    double GoalOrientationTolerance;
    string PlannerId;

    //Grasp Planning Member Variables
    std::vector<GraspPose> graspPoseList;
    gpd_ros::GraspConfigList candidates;
    double grasp_offset;
    double pregrasp_dist;

    //Manipulation Pose Member Variables
    std::vector<double> joint_group_positions;
    geometry_msgs::Pose target_pose;
    geometry_msgs::Pose place_pose;
    geometry_msgs::Vector3 orientation;

    //Flag Variables
    bool pose_success;

    //Gripper Functions
    void setGripper(trajectory_msgs::JointTrajectory& posture, double closeVal);
    void setGripper(double closeVal);

    //Functions
    void pick(std::vector<GraspPose> grasp_pose_list);
    void place(string place_pose);
    void pickAndPlace(std::vector<GraspPose> grasp_pose_list, string place_pose);
    void setPickingEEFPoseTarget(geometry_msgs::Pose grasp_position_pose, geometry_msgs::Pose grasp_orientation_pose);
    std::tuple<bool, moveit::planning_interface::MoveGroupInterface::Plan> plan(geometry_msgs::Pose grasp_position_pose, geometry_msgs::Pose grasp_orientation_pose);
    void addCollisions();

    //Helper Funcitons
    void storeGpdVals(gpd_ros::GraspConfigList candidates);
    void createPickingEEFPoseList();
    GraspPose createPickingEEFPose(gpd_ros::GraspConfig grasp_msg);

};

#endif // MANIPULATION_CLASS
