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
  this->gripper_command = nh.advertise<control_msgs::GripperCommandActionGoal>(gripperTopic, 10);
  this->robotiq_command = nh.advertise<robotiq_2f_gripper_control::Robotiq2FGripper_robot_output>("/Robotiq2FGripperRobotOutput", 10);

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

void Manipulation::removeCollision(string id)
{
  std::vector<std::string> object_ids;
  object_ids.push_back(id);
  this->planning_scene_ptr->removeCollisionObjects(object_ids);
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

  // cartesian trajectory planning parameters
  nh.getParam("/move_group/jump_threshold", jump_threshold);
  nh.getParam("/move_group/eef_step", eef_step);

  // end effector parameters
  nh.getParam("/end_effector/grasp_offset", grasp_offset);
  nh.getParam("/end_effector/pregrasp_dist", pregrasp_dist);
}


// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Gripper Functions
// Open and close gripper to different positions
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void Manipulation::setGripper(trajectory_msgs::JointTrajectory& posture, double closeVal)
{
  posture.joint_names.resize(2);
  posture.joint_names[0] = "finger_joint";

  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = closeVal;
  posture.points[0].time_from_start = ros::Duration(0.5);

  gripper_cmd.goal.command.position = closeVal;
  gripper_command.publish(gripper_cmd);
}

// Set the gripper to somewhere between fully opened and closed
// for robotiq_2f_85 open: 0, closed: ~0.8
void Manipulation::setGripper(double closeVal)
{
  int posVal = closeVal * 255;
  gripper_cmd.goal.command.position = closeVal;
  robotiq_cmd.rPR = posVal;
  gripper_command.publish(gripper_cmd);
  robotiq_command.publish(robotiq_cmd);
}

// plan a single target pose
bool Manipulation::plan(geometry_msgs::Pose grasp_pose, moveit::planning_interface::MoveGroupInterface::Plan& my_plan)
{
  move_group_ptr->setPoseTarget(grasp_pose);
  return move_group_ptr->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
}

// plan a cartesian path
double Manipulation::cartesianPlan(std::vector<geometry_msgs::Pose> pose_list, moveit::planning_interface::MoveGroupInterface::Plan& my_plan)
{
  moveit_msgs::RobotTrajectory trajectory;
  double success = move_group_ptr->computeCartesianPath(pose_list, eef_step, jump_threshold, trajectory);
  success *= 100;
  my_plan.trajectory_ = trajectory;
  ros::Duration(0.5).sleep();
  return success;
}

// Using helper function createPickingEEFPose graspPoses, plan and execute a picking maneuver with 3 individual movements
bool Manipulation::cartesianPick(std::vector<GraspPose> graspPose_list)
{
  std::vector<geometry_msgs::Pose> pre_and_grasp_pose_list;
  std::vector<geometry_msgs::Pose> retreat_pose_list;
  moveit::planning_interface::MoveGroupInterface::Plan pre_and_grasp_pose_plan;
  moveit::planning_interface::MoveGroupInterface::Plan retreat_pose_plan;

  bool temp_flag;

  unsigned long n = graspPose_list.size();
  for (unsigned long i = 0; i < n; ++i) {
    pre_and_grasp_pose_list.clear();
    pre_and_grasp_pose_list.push_back(graspPose_list[i].pre);
    pre_and_grasp_pose_list.push_back(graspPose_list[i].grasp);
    retreat_pose_list.clear();
    retreat_pose_list.push_back(graspPose_list[i].post);
    // get cartesian plan for pre and actual grasp, if good execute and close gripper
    double pre_and_grasp_success = cartesianPlan(pre_and_grasp_pose_list, pre_and_grasp_pose_plan);
    
    ROS_WARN("%f", pre_and_grasp_success);
    if (pre_and_grasp_success >= 100) {
      move_group_ptr->execute(pre_and_grasp_pose_plan);
      ros::Duration(1).sleep();
      setGripper(0);
      temp_flag = 1;
    }
    ros::Duration(1).sleep();
    // get cartesian plan for post grasp, if good execute
    if (temp_flag) {
      double post_success = cartesianPlan(retreat_pose_list, retreat_pose_plan);
      ROS_WARN("%f", post_success);

      if (post_success >= 100) {
        move_group_ptr->execute(retreat_pose_plan);
        ros::Duration(1).sleep();
        return true;
      }
    }
    ros::Duration(0.5).sleep();
  }
  return false;
}

void Manipulation::cartesianMove(std::vector<geometry_msgs::Pose> pose_list)
{
  unsigned long n = pose_list.size();
  for (unsigned long i = 0; i < n; ++i) {
    moveit::planning_interface::MoveGroupInterface::Plan pose_plan;
    moveit_msgs::RobotTrajectory trajectory;
    double success = move_group_ptr->computeCartesianPath(pose_list, eef_step, jump_threshold, trajectory, true, NULL);
    pose_plan.trajectory_ = trajectory;
    ros::Duration(0.5).sleep();
    success = success * 100;
    if (success == 100.0){
      if (move_group_ptr->execute(pose_plan)) {
        ros::Duration(0.5).sleep();
        break;
      }
    }
  }
}

void Manipulation::moveNamed(string poseName)
{
  move_group_ptr->setNamedTarget(poseName);
  move_group_ptr->move();
  ros::Duration(1.0).sleep();
}

void Manipulation::place(string placePose)
{
  move_group_ptr->setNamedTarget(placePose);
  move_group_ptr->move();
  ros::Duration(1.0).sleep();
  setGripper(0.0);
  ros::Duration(0.5).sleep();
}

bool Manipulation::pickandPlace(std::vector<GraspPose> graspPose_list, string placePose)
{
  bool success = cartesianPick(graspPose_list);

  //retract pose after pickup
  if (success)
  {
    this->place("retract");
  }
  return success;
}

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Moveit Tutorials Pick and Place Helper Functions
//
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void Manipulation::openGripper(trajectory_msgs::JointTrajectory& posture)
{
  posture.joint_names.resize(2);
  posture.joint_names[0] = "finger_joint";

  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 0.04;;
  posture.points[0].time_from_start = ros::Duration(0.5);
}

void Manipulation::closedGripper(trajectory_msgs::JointTrajectory& posture)
{
  posture.joint_names.resize(2);
  posture.joint_names[0] = "finger_joint";

  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 0.00;
  posture.points[0].time_from_start = ros::Duration(0.5);
}

void Manipulation::pick(vector<geometry_msgs::Pose> grasp_poses)
{
  std::vector<moveit_msgs::Grasp> grasps;
  grasps.clear();

  unsigned long n = grasp_poses.size();
  for (unsigned long i = 0; i < n; ++i) {
    grasps[i].grasp_pose.pose.orientation = grasp_poses[1].orientation;
    grasps[i].grasp_pose.pose.position = grasp_poses[1].position;

    grasps[i].pre_grasp_approach.direction.header.frame_id = "ee_link";
    grasps[i].pre_grasp_approach.direction.vector.y = 1.0;
    grasps[i].pre_grasp_approach.min_distance = pregrasp_dist;
    grasps[i].pre_grasp_approach.desired_distance = pregrasp_dist + 0.05;

    grasps[i].post_grasp_retreat.direction.header.frame_id = "ee_link";
    grasps[i].post_grasp_retreat.direction.vector.z = 1.0;
    grasps[i].post_grasp_retreat.min_distance = pregrasp_dist;
    grasps[i].post_grasp_retreat.desired_distance = pregrasp_dist + 0.05;

    openGripper(grasps[i].pre_grasp_posture);
    closedGripper(grasps[i].grasp_posture);
  }

  // For pick/place operations, the name of the support surface is used to specify the fact that attached objects are allowed to touch the support surface.
  //move_group_ptr->setSupportSurfaceName("table1");
  move_group_ptr->pick("object", grasps);
}

void Manipulation::place(geometry_msgs::Pose place_pose)
{
  //geometry_msgs::PoseStamped place_pose_stamped;
  //place_pose_stamped.pose = place_pose;
  //place_pose_stamped.header.frame_id = "ee_link";

  std::vector<moveit_msgs::PlaceLocation> place_location;

  place_location[0].place_pose.header.frame_id = "ee_link";
  tf2::Quaternion orientation;
  orientation.setRPY(0, 0, M_PI / 2);
  place_location[0].place_pose.pose.orientation = place_pose.orientation;
  place_location[0].place_pose.pose.position = place_pose.position;

  place_location[0].pre_place_approach.direction.header.frame_id = "ee_link";
  place_location[0].pre_place_approach.direction.vector.z = -1.0;
  place_location[0].pre_place_approach.min_distance = 0.095;
  place_location[0].pre_place_approach.desired_distance = 0.115;

  place_location[0].post_place_retreat.direction.header.frame_id = "ee_link";
  place_location[0].post_place_retreat.direction.vector.y = -1.0;
  place_location[0].post_place_retreat.min_distance = 0.1;
  place_location[0].post_place_retreat.desired_distance = 0.25;

  openGripper(place_location[0].post_place_posture);

  // place named object somewhere safe
  //move_group_ptr->place("object");

  // place named collision object on named collision surface
  //move_group_ptr->setSupportSurfaceName("table2");
  move_group_ptr->place("object", place_location);

  //move_group_ptr->place("object", place_pose_stamped);
}


void Manipulation::addCollisionObjects()
{
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(4);

  // Define the primitive and its dimensions.
  collision_objects[0].id = "table";
  collision_objects[0].header.frame_id = "map";
  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
  collision_objects[0].primitives[0].dimensions.resize(3);
  collision_objects[0].primitives[0].dimensions[0] = 0.62;
  collision_objects[0].primitives[0].dimensions[1] = 1.07;
  collision_objects[0].primitives[0].dimensions[2] = 0.82;

  // Define the pose of the table.
  collision_objects[0].primitive_poses.resize(1);
  collision_objects[0].primitive_poses[0].position.x = -1.51706802845;
  collision_objects[0].primitive_poses[0].position.y = 1.14764738083;
  collision_objects[0].primitive_poses[0].position.z = 0.42;

  //0.45, -0.58, 0.42
  //0,0,0.367,0.930

  collision_objects[0].primitive_poses[0].orientation.x = 0;
  collision_objects[0].primitive_poses[0].orientation.y = 0;
  collision_objects[0].primitive_poses[0].orientation.z = 0.924409835965;
  collision_objects[0].primitive_poses[0].orientation.w = 0.38140064915;

  collision_objects[0].operation = collision_objects[0].ADD;

  // Define the primitive and its dimensions.
  collision_objects[1].id = "table2";
  collision_objects[1].header.frame_id = "map";
  collision_objects[1].primitives.resize(1);
  collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[1].primitives[0].dimensions.resize(3);
  collision_objects[1].primitives[0].dimensions[0] = 1.1;
  collision_objects[1].primitives[0].dimensions[1] = 0.6;
  collision_objects[1].primitives[0].dimensions[2] = 1.15;

  // Define the pose of the table.
  collision_objects[1].primitive_poses.resize(1);
  collision_objects[1].primitive_poses[0].position.x = -1.92327606678;
  collision_objects[1].primitive_poses[0].position.y = -0.322113543749;
  collision_objects[1].primitive_poses[0].position.z = 0.42;

  collision_objects[1].primitive_poses[0].orientation.x = 0;
  collision_objects[1].primitive_poses[0].orientation.y = 0;
  collision_objects[1].primitive_poses[0].orientation.z = 0.924409835965;
  collision_objects[1].primitive_poses[0].orientation.w = 0.38140064915;
  


  //0.45, -0.58, 0.42
  //0,0,0.367,0.930
  collision_objects[1].operation = collision_objects[0].ADD;

  // Define the primitive and its dimensions.
  collision_objects[2].id = "head_box";
  collision_objects[2].header.frame_id = "head_pan_link";
  collision_objects[2].primitives.resize(1);
  collision_objects[2].primitives[0].type = collision_objects[2].primitives[0].BOX;
  collision_objects[2].primitives[0].dimensions.resize(3);
  collision_objects[2].primitives[0].dimensions[0] = 0.44;
  collision_objects[2].primitives[0].dimensions[1] = 0.55;
  collision_objects[2].primitives[0].dimensions[2] = 0.6;

  // Define the pose of the box.
  collision_objects[2].primitive_poses.resize(1);
  collision_objects[2].primitive_poses[0].position.x = 0;
  collision_objects[2].primitive_poses[0].position.y = 0;
  collision_objects[2].primitive_poses[0].position.z = 0.3;

  collision_objects[2].primitive_poses[0].orientation.x = 0;
  collision_objects[2].primitive_poses[0].orientation.y = 0;
  collision_objects[2].primitive_poses[0].orientation.z = 0;
  collision_objects[2].primitive_poses[0].orientation.w = 0;
  
  collision_objects[2].operation = collision_objects[0].ADD;

  /*
  // Define the primitive and its dimensions.
  collision_objects[3].id = "base_box";
  collision_objects[3].header.frame_id = "base_link";
  collision_objects[3].primitives.resize(1);
  collision_objects[3].primitives[0].type = collision_objects[2].primitives[0].BOX;
  collision_objects[3].primitives[0].dimensions.resize(3);
  collision_objects[3].primitives[0].dimensions[0] = 0.55;
  collision_objects[3].primitives[0].dimensions[1] = 0.55;
  collision_objects[3].primitives[0].dimensions[2] = 0.35;

  // Define the pose of the table.
  collision_objects[3].primitive_poses.resize(1);
  collision_objects[3].primitive_poses[0].position.x = 0;
  collision_objects[3].primitive_poses[0].position.y = 0;
  collision_objects[3].primitive_poses[0].position.z = 0.19;

  collision_objects[3].primitive_poses[0].orientation.x = 0;
  collision_objects[3].primitive_poses[0].orientation.y = 0;
  collision_objects[3].primitive_poses[0].orientation.z = 0;
  collision_objects[3].primitive_poses[0].orientation.w = 0;
  
  collision_objects[3].operation = collision_objects[0].ADD;
  */


  this->planning_scene_ptr->applyCollisionObjects(collision_objects);
}

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Manipulation-GPD Helper Functions
// Pass variables or otherwise wrap functions together
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

std::vector<GraspPose> Manipulation::createPickingEEFPoseList(gpd_ros::GraspConfigList candidates)
{
  std::vector<GraspPose> grasp_pose_list;
  grasp_pose_list.clear();
  unsigned long n = candidates.grasps.size();
  for (unsigned long i = 0; i < n; ++i) {
    grasp_pose_list.push_back(createPickingEEFPose(candidates.grasps[i]));
  }
  return grasp_pose_list;
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
  tf::poseTFToMsg(tf_grasp_odom, thisGrasp.grasp);

  tf::Transform tf_pregrasp_odom_(tf::Quaternion(0, 0, 0, 1), tf::Vector3(-pregrasp_dist, 0, 0));
  tf::Transform tf_pregrasp_odom = tf_grasp_odom * tf_pregrasp_odom_;
  tf::poseTFToMsg(tf_pregrasp_odom, thisGrasp.pre);

  tf::Transform tf_aftergrasp_odom_(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, -pregrasp_dist));
  tf::Transform tf_aftergrasp_odom = tf_grasp_odom * tf_aftergrasp_odom_;
  tf::poseTFToMsg(tf_aftergrasp_odom, thisGrasp.post);

  return thisGrasp;
}
