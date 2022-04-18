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
#include <fstream>
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
#include "navigation_class.hpp"
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/DeleteModel.h>
#include <robotiq_2f_gripper_control/Robotiq2FGripper_robot_output.h>

using namespace std;

typedef boost::shared_ptr<moveit::planning_interface::MoveGroupInterface> MoveGroupPtr;
typedef boost::shared_ptr<moveit::planning_interface::PlanningSceneInterface> PlanningScenePtr;
typedef boost::shared_ptr<tf::TransformListener> TransformListenerPtr;

struct GraspPose{
  geometry_msgs::Pose pre;
  geometry_msgs::Pose grasp;
  geometry_msgs::Pose post;
};

class Manipulation
{
  public:

    //Constructor
    Manipulation(ros::NodeHandle nodeHandle, std::string planning_group);
    void getParams(ros::NodeHandle nh);

    //Grasping Member Variables
    control_msgs::GripperCommandActionGoal gripper_cmd;
    robotiq_2f_gripper_control::Robotiq2FGripper_robot_output robotiq_cmd;
    ros::Publisher gripper_command;
    ros::Publisher robotiq_command;

    //Utility Class Pointers
    MoveGroupPtr move_group_ptr;
    PlanningScenePtr planning_scene_ptr;
    TransformListenerPtr transform_listener_ptr;

    //Grasp Planning Parameter Values
    double MaxVelocityScalingFactor;
    double PlanningTime;
    int NumPlanningAttempts;
    double GoalPositionTolerance;
    double GoalOrientationTolerance;
    string PlannerId;
    double jump_threshold;            // 0.5 default, one source uses 5.0 with good results, others use 0
    double eef_step;                  // 0.01 default (1 cm)
    double grasp_offset;
    double pregrasp_dist;

    //Gripper Functions
    void setGripper(trajectory_msgs::JointTrajectory& posture, double closeVal);
    void setGripper(double closeVal);
    void openGripper(trajectory_msgs::JointTrajectory& posture);
    void closedGripper(trajectory_msgs::JointTrajectory& posture);

    //Path Planning and Execution
    void moveNamed(string poseName);
    void pick(vector<geometry_msgs::Pose> grasp_poses);
    bool cartesianPick(std::vector<GraspPose> graspPose_list);
    void cartesianMove(std::vector<geometry_msgs::Pose> pose_list);
    void place(string place_pose);
    void place(geometry_msgs::Pose place_pose);
    bool pickandPlace(std::vector<GraspPose> graspPose_list, string place_pose);
    bool plan(geometry_msgs::Pose grasp_pose, moveit::planning_interface::MoveGroupInterface::Plan& my_plan);
    double cartesianPlan(std::vector<geometry_msgs::Pose> pose_list, moveit::planning_interface::MoveGroupInterface::Plan& my_plan);
    void removeCollision(string object_ids);

    //Grasp Position and Orientation Generation
    std::vector<GraspPose> createPickingEEFPoseList(gpd_ros::GraspConfigList candidates);
    GraspPose createPickingEEFPose(gpd_ros::GraspConfig grasp_msg);

    //Scene Functions
    void addCollisionObjects();

};

#endif // MANIPULATION_CLASS
