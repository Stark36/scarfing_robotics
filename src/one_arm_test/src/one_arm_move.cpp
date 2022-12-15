#include <iostream>
#include <cmath>

#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <moveit/robot_state/conversions.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <moveit_msgs/OrientationConstraint.h>

namespace rvt = rviz_visual_tools;
float head_diameter = 0.01;
float scarf_diameter = 0.2;
float scarf_center[7] = {0.5, 0, 0.5, 0, 0.7071068, 0, 0.7071068};
float target_pose_array[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

nav_msgs::Path path;

void circle_plan()
{
	float posex = 0.0;
	float posey = 0.0;
	float diameter = 0.0;
	int counter = 0;
	
	int segments = 36;
	int panes =floor((scarf_diameter-head_diameter)/head_diameter*2);
    
	for (int i = 0; i < panes; i++){
		segments = 36*floor(1+diameter/head_diameter*2);
		diameter += head_diameter/2;
		float count = 2*M_PI/segments;
		for (int j = 0; j < segments; j++){
	    		geometry_msgs::PoseStamped post;
	    		post.header.seq = counter;
	    		post.header.stamp = ros::Time::now();
	    		post.header.frame_id = "world";
	    		post.pose.position.x = diameter/2.0*sin(j*count) + scarf_center[0];
			post.pose.position.y = diameter/2.0*cos(j*count) + scarf_center[1];
			post.pose.position.z = scarf_center[2];
			path.poses.push_back(post);
			counter += 1;
		}
	}
}

int main(int argc, char** argv)
{
	// Node setup
	ros::init(argc, argv, "one_arm_move");
	ros::NodeHandle node_handle;
	ros::AsyncSpinner spinner(1);
	spinner.start();
	
	ros::Publisher path_pub = node_handle.advertise<nav_msgs::Path>("/get_path",1);
	path.header.frame_id="world";
	
	// Setup
	ros::Duration(1.0).sleep();
	static const std::string planning_grp = "ur5_arm"; //panda_arm, manipulator
	
	moveit::planning_interface::MoveGroupInterface move_group_interface(planning_grp);
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	const moveit::core::JointModelGroup* joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup(planning_grp);
	
	// rviz
	namespace rvt = rviz_visual_tools;
	moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
	visual_tools.deleteAllMarkers();

	// RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
	Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
	text_pose.translation().z() = 1.0;
	visual_tools.publishText(text_pose, "one_arm_move_test", rvt::WHITE, rvt::XLARGE);
	// Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
	visual_tools.trigger();
	// We can print the name of the reference frame for this robot.
	ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group_interface.getPlanningFrame().c_str());
	// We can also print the name of the end-effector link for this group.
	ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group_interface.getEndEffectorLink().c_str());
	// We can get a list of all the groups in the robot:
	ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
	std::copy(move_group_interface.getJointModelGroupNames().begin(),
	move_group_interface.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));

	circle_plan();
	path_pub.publish(path);

	// End goal
	geometry_msgs::Pose target_pose;
	target_pose.position.x = scarf_center[0];
        target_pose.position.y = scarf_center[1];
        target_pose.position.z = scarf_center[2];
	target_pose.orientation.x = scarf_center[3];
	target_pose.orientation.y = scarf_center[4];
	target_pose.orientation.z = scarf_center[5];
	target_pose.orientation.w = scarf_center[6];
	move_group_interface.setPoseTarget(target_pose);

	
	// Path constraint (changes planning parameters)
	/*
	moveit_msgs::OrientationConstraint ocm; 
	ocm.link_name = "wrist_3_link";
	ocm.header.frame_id = "base_link";
	ocm.orientation.w = 1.0;
	ocm.absolute_x_axis_tolerance = 0.1;
	ocm.absolute_y_axis_tolerance = 0.1;
	ocm.absolute_z_axis_tolerance = 0.1;
	ocm.weight = 1.0;
	
	moveit_msgs::Constraints test_constraints;
	test_constraints.orientation_constraints.push_back(ocm);
	move_group_interface.setPathConstraints(test_constraints);
	*/
	
	// Trajectory planning
	moveit::planning_interface::MoveGroupInterface::Plan move_plan;
	bool success = (move_group_interface.plan(move_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	
	ROS_INFO_NAMED("tutorial", "Visualizing plan (pose goal) %s", success ? "" : "FAILED");
	ROS_INFO_NAMED("tutorial", "Visualizing plan as trajectory line");
	visual_tools.publishAxisLabeled(target_pose, "pose");
	visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
	visual_tools.publishTrajectoryLine(move_plan.trajectory_, joint_model_group);
	visual_tools.trigger();
	
	move_group_interface.execute(move_plan);

	geometry_msgs::PoseStamped end_pose_reached;
	end_pose_reached = move_group_interface.getCurrentPose();
	ROS_INFO("Pose: %f, %f, %f", end_pose_reached.pose.position.x, end_pose_reached.pose.position.y, end_pose_reached.pose.position.z);
	ROS_INFO("Orientation: %f, %f, %f, %f", end_pose_reached.pose.orientation.x, end_pose_reached.pose.orientation.y, end_pose_reached.pose.orientation.z, end_pose_reached.pose.orientation.w);

	robot_state::RobotStatePtr end_state_reached = move_group_interface.getCurrentState();
	std::vector<double> joint_values;
	end_state_reached->copyJointGroupPositions(joint_model_group, joint_values);
	const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
	for (std::size_t i = 0; i < joint_names.size(); ++i)
	{
		ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
	}

	const Eigen::Isometry3d& end_effector_state = end_state_reached->getGlobalLinkTransform("wrist_3_link"); //panda_link8
	ROS_INFO_STREAM("Translation: \n" << end_effector_state.translation() << "\n");
	ROS_INFO_STREAM("Rotation: \n" << end_effector_state.rotation() << "\n");

	ros::shutdown();
	return 0;
}
