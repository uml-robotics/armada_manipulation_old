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
Manipulation::Manipulation(ros::NodeHandle nodeHandle, std::string planning_group)
{
    // start off by clearing the octomap in case it was previously occupied
    clearOctomap = nodeHandle.serviceClient<std_srvs::Empty>("/clear_octomap");

    // instantiate publisher for gripper commands
    nodeNamespace = nodeHandle.getNamespace();
    gripperTopic = nodeNamespace + "robotiq_2f_85_gripper_controller/gripper_cmd/goal";
    this->gripper_command = nodeHandle.advertise<control_msgs::GripperCommandActionGoal>(gripperTopic, 10);

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
void Manipulation::setGripper(double closeVal)
{
    this->gripper_cmd.goal.command.position = closeVal;
    gripper_command.publish(gripper_cmd);
}


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
}

void Manipulation::executeGrasp(GraspPose graspPose)
{
    set_target_pose_from_grasps(graspPose.pre, graspPose.actual);
    plan_and_move();
    ros::Duration(0.5).sleep();

    set_target_pose_from_grasps(graspPose.actual, graspPose.actual);
    plan_and_move();
    ros::Duration(0.5).sleep();

    setGripper(0.3);
    ros::Duration(2.0).sleep();

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
    // place(placePose);
    // Create a function to do this later
    move_group_ptr->setNamedTarget("retract");
    move_group_ptr->move();
    ros::Duration(0.5).sleep();

    setGripper(0.0);
    ros::Duration(2.0).sleep();
}

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Manipulation-GPD Helper Functions
// Pass variables or otherwise wrap functions together
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void Manipulation::store_gpd_vals(gpd_ros::GraspConfigList msg)
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

    // Find grasp actual, pre and after poses
    //
    // TODO: TWEAK THE VALUE ON LINE 219: the grasp offset along the z axis (distance from the object centerpoint) from the tooling mounting point to the actual grasping
    //    centerpoint; ie. the expected center of the fingerpad
    //
    // This value should effectively be the distance from the end of the robot's arm (the place where the gripper and other hardware is mounted to on the robot) and the point
    //    where the center of the object will be once the robot grasps it. The value will technically change depending on the object grasped so that will be developed further
    //    in the future with a function that considers the distance from the wrist to the center of the finger pads as a function of how closed the fingers are. This is only
    //    a need because the robotiq_2f_85 which we are currently using is not a "true" parallel gripper and the wider the object, the closer the finger pads are to the palm
    //
    // This value can remain static afer some tweaking for the time being since we need a general value and not perfect optomization immediately
    double grasp_point_dist = 0.095;

    tf::Transform tf_grasp_odom_(tf::Quaternion(0, 0, -M_PI/4 - M_PI/16, 1), tf::Vector3(0, 0, -grasp_point_dist));
    tf::Transform tf_grasp_odom = tf_base_odom * tf_grasp_base * tf_grasp_odom_;
    tf::poseTFToMsg(tf_grasp_odom, thisGrasp.actual);

    // TODO: TWEAK THE VALUE ON LINE 234: the pregrasp offset along the z axis (distance from actual grasp position)
    //
    // This value should be the distance from the actual grasp point along the z axis (relative to the gripper, so backward away from the object) that the gripper is told
    //    to use as a setpoint before grasping the object in order to be able to safely approach from outside the immediate object workspace.
    //    Due to current workflow, this value is also used as the "after" grasp, meaning the position that the robot brings the gripper to both enter and exit the immediate
    //    area surrounding the object we wish to grasp is the same both before and after the grasp.
    //
    // This value can also remain static after some tweaking, in the same way as the previous value this will actually depend more on other variables such as how many objects
    //    are in the scene, are there obstacles, etc.
    double pregrasp_offset_dist = 0.10;

    tf::Transform tf_pregrasp_odom_(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, -pregrasp_offset_dist));
    tf::Transform tf_pregrasp_odom = tf_grasp_odom * tf_pregrasp_odom_;
    tf::poseTFToMsg(tf_pregrasp_odom, thisGrasp.pre);

    tf::Transform tf_aftergrasp_odom_(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, 0));
    tf::Transform tf_aftergrasp_odom = tf_grasp_odom * tf_aftergrasp_odom_;
    tf::poseTFToMsg(tf_aftergrasp_odom, thisGrasp.after);

    return thisGrasp;
}
