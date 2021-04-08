// ********************************************************************************************
// Author: Brian Flynn;
// Test Engineer - NERVE Center @ UMASS Lowell
// manipulation_class.cpp
//
// manipulation_class function definitions
//
// Many of the following functions are direct or slightly modified blocks of code/functions
// from the Moveit! tutorials page (pick and place tutorial):
// http://docs.ros.org/en/kinetic/api/moveit_tutorials/html/doc/pick_place/pick_place_tutorial.html
// Check out the Moveit! tutorials for more good stuff
//
// ********************************************************************************************

#include "manipulation_class.hpp"

// ********************************************************************************************
// Constructors
// Create instance of Manipulation class and instantiate useful services, publishers
// and subscribers
// ********************************************************************************************

// MANIPULATION CLASS CONSTRUCTOR
Manipulation::Manipulation(ros::NodeHandle nodeHandle, std::string planning_group)
{
  // start off by clearing the octomap in case it was previously occupied
  clearOctomap = nodeHandle.serviceClient<std_srvs::Empty>("/clear_octomap");

  // establish all pointers
  planning_scene_ptr = PlanningScenePtr(
        new moveit::planning_interface::PlanningSceneInterface());
  move_group_ptr = MoveGroupPtr(
        new moveit::planning_interface::MoveGroupInterface(planning_group));
  transform_listener_ptr = TransformListenerPtr(
        new tf::TransformListener());
}

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Manipulation Gripper Functions
// Open and close gripper to different positions
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

// OPEN GRIPPER FUNCTION
void Manipulation::openGripper(trajectory_msgs::JointTrajectory& posture)
{
  posture.joint_names.resize(2);
  posture.joint_names[0] = "finger_joint";
  posture.joint_names[1] = "right_outer_knuckle_joint";

  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 0.00;
  posture.points[0].positions[1] = 0.00;
  posture.points[0].time_from_start = ros::Duration(0.5);
}

// CLOSE GRIPPER FUNCTION
void Manipulation::closeGripper(trajectory_msgs::JointTrajectory& posture)
{
  posture.joint_names.resize(2);
  posture.joint_names[0] = "finger_joint";
  posture.joint_names[1] = "right_outer_knuckle_joint";

  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 0.78;
  posture.points[0].positions[1] = 0.78;
  posture.points[0].time_from_start = ros::Duration(0.5);
}

// CLOSEVAR GRIPPER FUNCTION
// set the gripper to somewhere between fully opened and closed
void Manipulation::closeVarGripper(trajectory_msgs::JointTrajectory& posture, double closeVal)
{
  posture.joint_names.resize(2);
  posture.joint_names[0] = "finger_joint";
  posture.joint_names[1] = "right_outer_knuckle_joint";

  // Don't let val fall above/below max/min values
  if (closeVal >= 0.78)
  {
      closeVal = 0.78;
  } else if (closeVal <= 0.00)
  {
      closeVal = 0.00;
  }

  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = closeVal;
  posture.points[0].positions[1] = closeVal;
  posture.points[0].time_from_start = ros::Duration(0.5);
}

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Manipulation Pick and Place Functions
// Control picking and placing of objects - also approach and retreat
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

// Function for creating end-effector poses from GPD msg.
void Manipulation::createPickingEEFPose(gpd::GraspConfig grasp_msg)
{
  tf::StampedTransform tf_base_odom;
  tf::StampedTransform tf_hand_odom;
  tf::Matrix3x3 rot_matrix_grasp_base(-grasp_msg.axis.x, grasp_msg.binormal.x, grasp_msg.approach.x,
                                      -grasp_msg.axis.y, grasp_msg.binormal.y, grasp_msg.approach.y,
                                      -grasp_msg.axis.z, grasp_msg.binormal.z, grasp_msg.approach.z);

  tf::Vector3 tr_grasp_base(grasp_msg.bottom.x, grasp_msg.bottom.y, grasp_msg.bottom.z);
  tf::Transform tf_grasp_base(rot_matrix_grasp_base, tr_grasp_base);

  try
  {
    transform_listener_ptr->waitForTransform("base_link", "base_link", ros::Time(0), ros::Duration(3.0) );
    transform_listener_ptr->lookupTransform("base_link", "base_link", ros::Time(0), tf_base_odom);
  } catch (tf::TransformException err)
  {
    ROS_ERROR("%s", err.what());
  }

  // Find grasp pose
  tf::Transform tf_grasp_odom_(tf::Quaternion(0, 0, M_PI/2, 1), tf::Vector3(0, 0, -0.148));
  tf::Transform tf_grasp_odom = tf_base_odom * tf_grasp_base * tf_grasp_odom_;
  tf::poseTFToMsg(tf_grasp_odom, grasp_poses.actual);

  // Find pre-grasp pose
  tf::Transform tf_pregrasp_odom_(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, -0.08));
  tf::Transform tf_pregrasp_odom = tf_grasp_odom * tf_pregrasp_odom_;
  tf::poseTFToMsg(tf_pregrasp_odom, grasp_poses.pre);

  // Find after-grasp pose
  grasp_poses.after = grasp_poses.actual;
  grasp_poses.after.position.z = grasp_poses.after.position.z + 0.15;
}

void Manipulation::set_target_pose(geometry_msgs::Pose graspPose)
{
    q.setRPY(graspPose.orientation.x - M_PI, graspPose.orientation.y, graspPose.orientation.z);
    q.normalize();
    target_pose.orientation = tf2::toMsg(q);
    target_pose.position.x = graspPose.position.x;
    target_pose.position.y = graspPose.position.y;
    target_pose.position.z = graspPose.position.z;
}

void Manipulation::go_to_poses_test(geometry_msgs::Pose graspPose)
{
    target_pose.orientation = grasp_poses.actual.orientation;
    target_pose.position.x = graspPose.position.x;
    target_pose.position.y = graspPose.position.y;
    target_pose.position.z = graspPose.position.z;

    move_group_ptr->setPoseTarget(target_pose);
    move_group_ptr->setGoalPositionTolerance(0.001);
    move_group_ptr->setGoalOrientationTolerance(0.002);
    move_group_ptr->setPlanningTime(5.0);
    move_group_ptr->setNumPlanningAttempts(30);
    if(move_group_ptr->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
        move_group_ptr->move();
    }
}

void Manipulation::pick_and_place(GraspPose graspPoses)
{
  go_to_poses_test(graspPoses.pre);
  ros::Duration(1.0).sleep();

  go_to_poses_test(graspPoses.actual);
  ros::Duration(1.0).sleep();

  go_to_poses_test(graspPoses.pre);
  ros::Duration(1.0).sleep();
}

void Manipulation::pick(GraspPose graspPoses)
{
    // TODO
}

void Manipulation::place(GraspPose graspPoses)
{
    // TODO
}

void Manipulation::pick_and_place()
{
    // TODO?
}

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Manipulation Helper Functions
// Pass variables or otherwise wrap functions together
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void Manipulation::store_gpd_vals(gpd::GraspConfigList msg)
{
  ROS_INFO("storing gpd vals");
  candidates = msg;
  if(candidates.grasps.size() == 0)
  {
    ROS_INFO("error: grasp list is empty");
  }
}

void Manipulation::addCollisionObjects(GraspPose graspPoses)
{
  // Create vector to hold 3 collision objects.
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(2);

  collision_objects[0].id = "object";
  collision_objects[0].header.frame_id = "world";

  /* Define the primitive and its dimensions. */
  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
  collision_objects[0].primitives[0].dimensions.resize(3);
  collision_objects[0].primitives[0].dimensions[0] = 0.01;
  collision_objects[0].primitives[0].dimensions[1] = 0.01;
  collision_objects[0].primitives[0].dimensions[2] = 0.01;

  collision_objects[0].primitive_poses.resize(1);
  collision_objects[0].primitive_poses[0].position.x = graspPoses.actual.position.x;
  collision_objects[0].primitive_poses[0].position.y = graspPoses.actual.position.y;
  collision_objects[0].primitive_poses[0].position.z = graspPoses.actual.position.z;

  collision_objects[0].operation = collision_objects[0].ADD;

  // Add the second table where we will be placing the cube.
  collision_objects[1].id = "table";
  collision_objects[1].header.frame_id = "world";

  /* Define the primitive and its dimensions. */
  collision_objects[1].primitives.resize(1);
  collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[1].primitives[0].dimensions.resize(3);
  collision_objects[1].primitives[0].dimensions[0] = 0.3;
  collision_objects[1].primitives[0].dimensions[1] = 0.3;
  collision_objects[1].primitives[0].dimensions[2] = 0.3;

  collision_objects[1].primitive_poses.resize(1);
  collision_objects[1].primitive_poses[0].position.x = -0.45;
  collision_objects[1].primitive_poses[0].position.y = -0.45;
  collision_objects[1].primitive_poses[0].position.z = 0.67;

  collision_objects[1].operation = collision_objects[1].ADD;

  planning_scene_ptr->applyCollisionObjects(collision_objects);
}
