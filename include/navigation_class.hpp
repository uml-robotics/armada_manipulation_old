// ********************************************************************************************
// Author: Kevin Yassini;
// navigation_class.hpp
// ********************************************************************************************

#ifndef NAVIGATION_CLASS_HPP
#define NAVIGATION_CLASS_HPP

#include <actionlib/client/simple_action_client.h>
#include <control_msgs/PointHeadActionGoal.h>
#include <geometry_msgs/Pose.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>

using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
typedef boost::shared_ptr<MoveBaseClient> MoveBasePtr;

class Navigation
{
  public:
    MoveBasePtr move_base_ptr;

    //Constructor
    Navigation(ros::NodeHandle nodeHandle);

    //Navigation helper functions
    void setHead();
    void sendGoal(double x, double y, double ang);  

    //Fetch's Head member variables
    control_msgs::PointHeadActionGoal head_cmd;
    ros::Publisher head_command;
};

#endif // NAVIGATION_CLASS
