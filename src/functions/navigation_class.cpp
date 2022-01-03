// ********************************************************************************************
// Author: Kevin Yassini;
// navigation_class.cpp
//
// navigation_class function definitions
// Methods for mobile manipulation
// ********************************************************************************************

#include "navigation_class.hpp"

// NAVIGATION_CLASS CONSTRUCTOR
Navigation::Navigation(ros::NodeHandle nh)
{
  string headTopic = nh.getNamespace() + "/head_controller/point_head/goal"; // For moving fetch's head
  this->head_command = nh.advertise<control_msgs::PointHeadActionGoal>(headTopic, 10);

  move_base_ptr = MoveBasePtr(
      new MoveBaseClient("move_base", true));
  while (!move_base_ptr->waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO("Waiting for the move_base action server to come up...");
  }
}

// Navigate to a specific pose on the map.
void Navigation::sendGoal(double x, double y, double ang)
{
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  tf2::Quaternion q;
  q.setRPY(0, 0, ang);
  goal.target_pose.pose.position.x = x;
  goal.target_pose.pose.position.y = y;

  goal.target_pose.pose.orientation.x = q[0];
  goal.target_pose.pose.orientation.y = q[1];
  goal.target_pose.pose.orientation.z = q[2];
  goal.target_pose.pose.orientation.w = q[3];

  move_base_ptr->sendGoal(goal);
  move_base_ptr->waitForResult();
}

// Set Fetch's head to aim at a specific point w.r.t the map.
void Navigation::setHead()
{
  this->head_cmd.goal.target.header.frame_id = "map";
  this->head_cmd.goal.target.point.x = 0.45;
  this->head_cmd.goal.target.point.y = -0.55;
  this->head_cmd.goal.target.point.z = 0.75;

  head_command.publish(head_cmd);
}