// ********************************************************************************************
// Author: Brian Flynn;
// Test Engineer - NERVE Center @ UMASS Lowell
// manipulation_class.cpp
//
// manipulation_class function definitions
//
// Edit this later for more info...
//
// ********************************************************************************************

#include "manipulation_class.hpp"

// ********************************************************************************************
// Constructors
// Create instance of Manipulation class and instantiate useful services, publishers
// and subscribers
// ********************************************************************************************

// MANIPULATION CLASS CONSTRUCTOR
Manipulation::Manipulation(ros::NodeHandle nh, std::string planning_group)
{
  // Instantiate publisher for gripper commands
  string gripper_name;
  nh.getParam("/end_effector/name", gripper_name);
  string gripperTopic = nh.getNamespace() + "/gripper_controller/gripper_action/goal";  // Fetch has a different gripper action topic
  string headTopic = nh.getNamespace() + "/head_controller/point_head/goal";  // For moving fetch's head
  this->gripper_command = nh.advertise<control_msgs::GripperCommandActionGoal>(gripperTopic, 10);
  this->head_command = nh.advertise<control_msgs::PointHeadActionGoal>(headTopic, 10);
  // Establish all pointers
  planning_scene_ptr = PlanningScenePtr(
        new moveit::planning_interface::PlanningSceneInterface());
  move_group_ptr = MoveGroupPtr(
        new moveit::planning_interface::MoveGroupInterface(planning_group));
  transform_listener_ptr = TransformListenerPtr(
        new tf::TransformListener());

  // Initialize speed values, move robot into place
  getParams(nh);

  move_group_ptr->setNamedTarget("retract");
  //move_group_ptr->move(); // Fetch does not have a retract target yet.
}

void Manipulation::getParams(ros::NodeHandle nh)
{
  // move_group parameters
  nh.getParam("/move_group/MaxVelocityScalingFactor", MaxVelocityScalingFactor);
  nh.getParam("/move_group/PlanningTime", PlanningTime);
  nh.getParam("/move_group/NumPlanningAttempts", NumPlanningAttempts);
  nh.getParam("/move_group/GoalPositionTolerance", GoalPositionTolerance);
  nh.getParam("/move_group/GoalOrientationTolerance", GoalOrientationTolerance);
  nh.getParam("/move_group/PlannerId", PlannerId);

  move_group_ptr->setMaxVelocityScalingFactor(MaxVelocityScalingFactor);
  move_group_ptr->setPlanningTime(PlanningTime);
  move_group_ptr->setNumPlanningAttempts(NumPlanningAttempts);
  move_group_ptr->setGoalPositionTolerance(GoalPositionTolerance);
  move_group_ptr->setGoalOrientationTolerance(GoalOrientationTolerance);
  move_group_ptr->setPlannerId(PlannerId);

  // end effector parameters
  nh.getParam("/end_effector/grasp_offset", grasp_offset);
  nh.getParam("/end_effector/pregrasp_dist", pregrasp_dist);
}

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Fetch Functions
// Set Fetch's head to aim at a specific point w.r.t its base.
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void Manipulation::setHead()
{ 
  this->head_cmd.goal.target.header.frame_id = "base_link";
  this->head_cmd.goal.target.point.x = 0.6;
  this->head_cmd.goal.target.point.y = 0.1;
  this->head_cmd.goal.target.point.z = 0.7;

  head_command.publish(head_cmd);
}

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Gripper Functions
// Open and close gripper to different positions
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


// Set the gripper to somewhere between fully opened and closed
// for robotiq_2f_85 open: 0, closed: ~0.8
void Manipulation::setGripper(double closeVal)
{
  this->gripper_cmd.goal.command.position = closeVal;
  gripper_command.publish(gripper_cmd);
}

// Given some graspPoses, set target pose
void Manipulation::setPickingEEFPoseTarget(geometry_msgs::Pose graspPositionPose, geometry_msgs::Pose graspOrientationPose)
{
  //geometry_msgs::Pose target_pose;

  target_pose.orientation = graspOrientationPose.orientation;
  target_pose.position.x = graspPositionPose.position.x;
  target_pose.position.y = graspPositionPose.position.y;
  target_pose.position.z = graspPositionPose.position.z;
}

// plan
std::tuple<bool, moveit::planning_interface::MoveGroupInterface::Plan> Manipulation::plan(geometry_msgs::Pose graspPositionPose, geometry_msgs::Pose graspOrientationPose)
{
  setPickingEEFPoseTarget(graspPositionPose, graspOrientationPose);
  move_group_ptr->setPoseTarget(target_pose);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  return std::make_tuple(move_group_ptr->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS, my_plan);
}

// Using helper function createPickingEEFPose graspPoses, plan and execute a picking maneuver with 3 individual movements
void Manipulation::pick(std::vector<GraspPose> graspPoseList)
{
  unsigned long n = graspPoseList.size();
  if (n >10) {
    // limit this value for now
    n = 10;
  }
  for (unsigned long i = 0; i < n; ++i) {
    std::tuple<bool,moveit::planning_interface::MoveGroupInterface::Plan> grasp_plan_pre = plan(graspPoseList[i].pre, graspPoseList[i].actual);
    std::tuple<bool,moveit::planning_interface::MoveGroupInterface::Plan> grasp_plan_actual = plan(graspPoseList[i].actual, graspPoseList[i].actual);

    if (std::get<0>(grasp_plan_pre) && std::get<0>(grasp_plan_actual)) {
      move_group_ptr->execute(std::get<1>(grasp_plan_pre));
      ros::Duration(0.5).sleep();
      std::tuple<bool,moveit::planning_interface::MoveGroupInterface::Plan> grasp_plan_actual = plan(graspPoseList[i].actual, graspPoseList[i].actual);
      if (std::get<0>(grasp_plan_actual)) {
        move_group_ptr->execute(std::get<1>(grasp_plan_actual));
        ros::Duration(0.5).sleep();
        setGripper(0);  // Only 0 or 1 for Fetch's gripper
        ros::Duration(2.0).sleep();
        std::tuple<bool,moveit::planning_interface::MoveGroupInterface::Plan> grasp_plan_pre = plan(graspPoseList[i].pre, graspPoseList[i].actual);
        if (std::get<0>(grasp_plan_pre)) {
          move_group_ptr->execute(std::get<1>(grasp_plan_pre));
          ros::Duration(0.5).sleep();
        }
      }
      break;
    }
  }
}

void Manipulation::place(string placePose)
{
  move_group_ptr->setNamedTarget(placePose);
  move_group_ptr->move();
  ros::Duration(0.5).sleep();
}

void Manipulation::pickAndPlace(std::vector<GraspPose> graspPoseList, string placePose)
{
  pick(graspPoseList);
  place(placePose);

  setGripper(1);  // Only 0 or 1 for Fetch's gripper
  ros::Duration(2.0).sleep();
}

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Manipulation-GPD Helper Functions
// Pass variables or otherwise wrap functions together
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void Manipulation::storeGpdVals(gpd_ros::GraspConfigList msg)
{
  candidates = msg;
  if(candidates.grasps.size() == 0) {
    ROS_WARN_STREAM("Grasp Candidates List Contains No Values");
  }
}

void Manipulation::createPickingEEFPoseList()
{
  unsigned long n = candidates.grasps.size();
  graspPoseList.resize(n);
  for (unsigned long i = 0; i < n; ++i) {
    graspPoseList[i] = createPickingEEFPose(candidates.grasps[i]);
  }
}

GraspPose Manipulation::createPickingEEFPose(gpd_ros::GraspConfig grasp_msg)
{
  GraspPose thisGrasp;
  tf::Matrix3x3 rot_matrix_grasp_base(-grasp_msg.axis.x, grasp_msg.binormal.x, grasp_msg.approach.x,
                                      -grasp_msg.axis.y, grasp_msg.binormal.y, grasp_msg.approach.y,
                                      -grasp_msg.axis.z, grasp_msg.binormal.z, grasp_msg.approach.z);

  tf::Vector3 tr_grasp_base(grasp_msg.position.x, grasp_msg.position.y, grasp_msg.position.z);
  tf::Transform tf_grasp_base(rot_matrix_grasp_base, tr_grasp_base);
  tf::StampedTransform tf_base_odom;

  try {
    transform_listener_ptr->waitForTransform("base_link", "base_link", ros::Time(0), ros::Duration(3.0) );
    transform_listener_ptr->lookupTransform("base_link", "base_link", ros::Time(0), tf_base_odom);
  } catch (tf::TransformException err) {
    ROS_ERROR("%s", err.what());
  }

  // Fetch specific rotaion values
  tf::Transform tf_grasp_odom_(tf::Quaternion(-0.5, -0.5, -0.5, 0.5), tf::Vector3(0, 0, -grasp_offset));
  tf::Transform tf_grasp_odom = tf_base_odom * tf_grasp_base * tf_grasp_odom_;
  tf::poseTFToMsg(tf_grasp_odom, thisGrasp.actual);

  tf::Transform tf_pregrasp_odom_(tf::Quaternion(0, 0, 0, 1), tf::Vector3(-pregrasp_dist, 0, 0));
  tf::Transform tf_pregrasp_odom = tf_grasp_odom * tf_pregrasp_odom_;
  tf::poseTFToMsg(tf_pregrasp_odom, thisGrasp.pre);

  tf::Transform tf_aftergrasp_odom_(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, 0));
  tf::Transform tf_aftergrasp_odom = tf_grasp_odom * tf_aftergrasp_odom_;
  tf::poseTFToMsg(tf_aftergrasp_odom, thisGrasp.after);

  return thisGrasp;
}
