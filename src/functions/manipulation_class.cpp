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
// Gripper Functions
// Open and close gripper to different positions
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

// CLOSEVAR GRIPPER FUNCTION
// set the gripper to somewhere between fully opened and closed
// for robotiq_2f_85 open: 0, closed: ~0.8
void Manipulation::setGripper(trajectory_msgs::JointTrajectory& posture, double closeVal)
{
    posture.joint_names.resize(2);
    posture.joint_names[0] = "finger_joint";
    posture.joint_names[1] = "right_outer_knuckle_joint";

    // Don't let val fall above/below max/min values
    if (closeVal >= 0.78) {
        closeVal = 0.78;
    } else if (closeVal <= 0.00) {
        closeVal = 0.00;
    }

    posture.points.resize(1);
    posture.points[0].positions.resize(2);
    posture.points[0].positions[0] = closeVal;
    posture.points[0].positions[1] = closeVal;
    posture.points[0].time_from_start = ros::Duration(0.5);
}

/*
void Manipulation::setGripperJointPos(double closeVal)
{

}
*/

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Manipulation Pick and Place Functions
// Control picking and placing of objects - also approach and retreat
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

// Given a particular geometry_msgs::Pose with unnormalized quaternion, set target_pose
void Manipulation::set_target_pose_rpy(geometry_msgs::Pose graspPose)
{
    q.setRPY(graspPose.orientation.x - M_PI, graspPose.orientation.y, graspPose.orientation.z);
    q.normalize();
    target_pose.orientation = tf2::toMsg(q);
    target_pose.position.x = graspPose.position.x;
    target_pose.position.y = graspPose.position.y;
    target_pose.position.z = graspPose.position.z;
}

// Given some graspPoses, set target pose
void Manipulation::set_target_pose_from_grasps(geometry_msgs::Pose graspPositionPose, geometry_msgs::Pose graspOrientationPose)
{
    target_pose.orientation = graspOrientationPose.orientation;
    target_pose.position.x = graspPositionPose.position.x;
    target_pose.position.y = graspPositionPose.position.y;
    target_pose.position.z = graspPositionPose.position.z;
}

void Manipulation::plan_and_move()
{
    move_group_ptr->setPoseTarget(target_pose);
    move_group_ptr->setGoalPositionTolerance(0.001);
    move_group_ptr->setGoalOrientationTolerance(0.002);
    move_group_ptr->setPlanningTime(5.0);
    move_group_ptr->setNumPlanningAttempts(30);
    if(move_group_ptr->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
        move_group_ptr->move();
    }
}

bool Manipulation::plan(geometry_msgs::Pose graspPositionPose, geometry_msgs::Pose graspOrientationPose)
{
    ROS_INFO("planning ...");
    set_target_pose_from_grasps(graspPositionPose, graspOrientationPose);

    move_group_ptr->setPoseTarget(target_pose);
    move_group_ptr->setGoalPositionTolerance(0.001);
    move_group_ptr->setGoalOrientationTolerance(0.002);
    move_group_ptr->setPlanningTime(5.0);
    move_group_ptr->setNumPlanningAttempts(30);

    return (move_group_ptr->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
}

// Using helper function createPickingEEFPose graspPoses, plan and execute a picking maneuver with 3 individual movements
void Manipulation::pick(std::vector<GraspPose> graspPoseList)
{
    int n = graspPoseList.size();
    if (n >10) {
        // limit this value for now
        n = 10;
    }
    for (int i = 0; i < n; ++i) {
        if (plan(graspPoseList[i].pre, graspPoseList[i].actual) && plan(graspPoseList[i].actual, graspPoseList[i].actual)) {
            executeGrasp(graspPoseList[i]);
            ROS_INFO("that was grasp candidate %f ...", i + 1);
            ros::Duration(0.5).sleep();
            break;
        }
    }

    /*
     * All candidates failed planning
     *
     * do something about it here, set flag so program can loop
     */
}

void Manipulation::executeGrasp(GraspPose graspPose)
{
    set_target_pose_from_grasps(graspPose.pre, graspPose.actual);
    plan_and_move();
    ros::Duration(0.5).sleep();

    set_target_pose_from_grasps(graspPose.actual, graspPose.actual);
    plan_and_move();
    ros::Duration(0.5).sleep();

    set_target_pose_from_grasps(graspPose.pre, graspPose.actual);
    plan_and_move();
    ros::Duration(0.5).sleep();
}

void Manipulation::place(geometry_msgs::Pose placePose)
{
    /*
     * TODO:
     *
     * set_target_pose(some_known_position);
     * plan_and_move
     */
    set_target_pose_rpy(placePose);
    plan_and_move();
}

void Manipulation::pick_and_place(std::vector<GraspPose> graspPoseList, geometry_msgs::Pose placePose)
{
    pick(graspPoseList);
    place(placePose);
}

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Manipulation-GPD Helper Functions
// Pass variables or otherwise wrap functions together
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void Manipulation::store_gpd_vals(gpd::GraspConfigList msg)
{
    ROS_INFO("storing gpd vals");
    candidates = msg;
    if(candidates.grasps.size() == 0) {
        ROS_INFO("error: grasp list is empty");
    }
}

void Manipulation::createPickingEEFPoseList()
{
    int n = candidates.grasps.size();
    graspPoseList.resize(n);
    for (int i = 0; i < n; ++i) {
        graspPoseList[i] = createPickingEEFPose(candidates.grasps[i]);
    }
    ROS_INFO("grasp pose list generated");
}

GraspPose Manipulation::createPickingEEFPose(gpd::GraspConfig grasp_msg)
{
    GraspPose thisGrasp;
    tf::Matrix3x3 rot_matrix_grasp_base(-grasp_msg.axis.x, grasp_msg.binormal.x, grasp_msg.approach.x,
                                      -grasp_msg.axis.y, grasp_msg.binormal.y, grasp_msg.approach.y,
                                      -grasp_msg.axis.z, grasp_msg.binormal.z, grasp_msg.approach.z);

    tf::Vector3 tr_grasp_base(grasp_msg.bottom.x, grasp_msg.bottom.y, grasp_msg.bottom.z);
    tf::Transform tf_grasp_base(rot_matrix_grasp_base, tr_grasp_base);
    tf::StampedTransform tf_base_odom;

    try {
        transform_listener_ptr->waitForTransform("base_link", "base_link", ros::Time(0), ros::Duration(3.0) );
        transform_listener_ptr->lookupTransform("base_link", "base_link", ros::Time(0), tf_base_odom);
    } catch (tf::TransformException err) {
        ROS_ERROR("%s", err.what());
    }

    // Find grasp actual, pre and after poses
    tf::Transform tf_grasp_odom_(tf::Quaternion(0, 0, -M_PI/4 - M_PI/16, 1), tf::Vector3(0, 0, -0.148));
    tf::Transform tf_grasp_odom = tf_base_odom * tf_grasp_base * tf_grasp_odom_;
    tf::poseTFToMsg(tf_grasp_odom, thisGrasp.actual);

    tf::Transform tf_pregrasp_odom_(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, -0.08));
    tf::Transform tf_pregrasp_odom = tf_grasp_odom * tf_pregrasp_odom_;
    tf::poseTFToMsg(tf_pregrasp_odom, thisGrasp.pre);

    tf::Transform tf_aftergrasp_odom_(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, 0));
    tf::Transform tf_aftergrasp_odom = tf_grasp_odom * tf_aftergrasp_odom_;
    tf::poseTFToMsg(tf_aftergrasp_odom, thisGrasp.after);

    return thisGrasp;
}
