#include <iostream>
#include <cmath>
#include <vector>
#include <algorithm>

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

#include <moveit_msgs/ExecuteTrajectoryActionFeedback.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include <nav_msgs/Path.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point32.h>
#include <moveit_msgs/OrientationConstraint.h>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <geometry_msgs/Point32.h>


using namespace std;

// Scarfing class
class scarf{

	private:
	int count;
	float end_eff_offset = 0.2;
	geometry_msgs::Pose target_pose;
	nav_msgs::Path path;
	bool move_wall = false;
	bool cartesian_start = false;
	std_msgs::Bool is_scarf_execute_bool;
	
	std::string planning_grp;
	moveit::planning_interface::MoveGroupInterface *move_group_interface;
	moveit::planning_interface::PlanningSceneInterface *planning_scene_interface;
	const moveit::core::JointModelGroup *joint_model_group;
	std::vector<moveit_msgs::CollisionObject> collision_objects;
	
	public:
	ros::NodeHandle node_handle;
	ros::Subscriber temp_sub;
	ros::Subscriber path_sub;
	ros::Subscriber exec_feedback_sub;
	ros::Publisher is_scarf_execute_pub;
	
	scarf(std::string grp, moveit::planning_interface::MoveGroupInterface *move0, moveit::planning_interface::PlanningSceneInterface *scene0){
		count = 0;
		path.header.frame_id="world";
		
		// Set up of subscribers and publishers
		temp_sub = node_handle.subscribe("/temp",1, &scarf::temp, this);
		path_sub = node_handle.subscribe("/get_path",1, &scarf::path_callback, this);
		exec_feedback_sub = node_handle.subscribe("/execute_trajectory/feedback",1, &scarf::exec_feedback_callback, this);
		is_scarf_execute_pub = node_handle.advertise<std_msgs::Bool>("/is_scarf_execute",1);
		
		// Linking to moveit group and planning scene interface
		planning_grp = grp;
		move_group_interface = move0;
		(*move_group_interface).setPlanningTime(60.0);
		planning_scene_interface = scene0;
		joint_model_group = (*move_group_interface).getCurrentState()->getJointModelGroup(planning_grp);
		
		
		// Adding floor as collision object to planning scene to account for floor during path planning
		moveit_msgs::CollisionObject collision_object;
		collision_object.header.frame_id = (*move_group_interface).getPlanningFrame();
		// Unused pose field in moveit_msgs::CollisionObject, set to remove warning
		collision_object.pose.orientation.w = 1.0;
		collision_object.pose.position.x = 0.0;
		
		collision_object.id = "floor";
		shape_msgs::SolidPrimitive primitive;
		primitive.type = primitive.BOX;
		primitive.dimensions.resize(3);
		primitive.dimensions[primitive.BOX_X] = 3;
		primitive.dimensions[primitive.BOX_Y] = 3;
		primitive.dimensions[primitive.BOX_Z] = 0.05;
		
		geometry_msgs::Pose floor_pose;
		floor_pose.orientation.x = 0.0;
		floor_pose.orientation.y = 0.0;
		floor_pose.orientation.z = 0.0;
		floor_pose.orientation.w = 1.0;
		floor_pose.position.x = 0.0;
		floor_pose.position.y = 0.0;
		floor_pose.position.z = 0.0;

		collision_object.primitives.push_back(primitive);
		collision_object.primitive_poses.push_back(floor_pose);
		collision_object.operation = collision_object.ADD;

		collision_objects.push_back(collision_object);
		planning_scene_interface->addCollisionObjects(collision_objects);
		
	}
	
	// Subscriber template
	void temp(const std_msgs::Int64& msg){
		ROS_INFO_STREAM("Temp: " << msg.data << "\n");
	}
	
	// Subscriber to obtain region of path plan from evaluator
	void path_callback(const nav_msgs::Path& input){
		path.poses = input.poses;
		ROS_INFO_STREAM("Received region of path plan from evaluator " << "\n");
		execute_path();
	}
	
	// Subscriber to check if manipulator is in motion or if it has stopped
	void exec_feedback_callback(const moveit_msgs::ExecuteTrajectoryActionFeedback& feedback){
		ROS_INFO_STREAM("Feedback status: " << feedback.feedback.state <<"\n");
		if (feedback.feedback.state == "MONITOR"){
			move_wall = true;
		}else{
			move_wall = false;
		}	
	}
	
	void execute_path(){
		std::vector<geometry_msgs::Pose> waypoints;
		int n_waypoints = path.poses.size();
		ROS_INFO_STREAM("Number of waypoints: " << n_waypoints << "\n");
		//Parallel to Z-axis of the world frame
		path.poses[0].pose.orientation.x = 0.0;
		path.poses[0].pose.orientation.y = 0.7071068;
		path.poses[0].pose.orientation.z = 0.0;
		path.poses[0].pose.orientation.w = 0.7071068;
		path.poses[0].pose.position.z += end_eff_offset;
		set_goal(path.poses[0].pose);
		move_to_goal();
		
		for (int i = 1; i < n_waypoints; i++){
			path.poses[i].pose.orientation.x = 0.0;
			path.poses[i].pose.orientation.y = 0.7071068;
			path.poses[i].pose.orientation.z = 0.0;
			path.poses[i].pose.orientation.w = 0.7071068;
			path.poses[i].pose.position.z += end_eff_offset;
			waypoints.push_back(path.poses[i].pose);
		}
		moveit_msgs::RobotTrajectory trajectory;
		const double jump_threshold = 0.0;
		const double eef_step = 0.005;
		double fraction = (*move_group_interface).computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
		// Removal of last trajectory point due to non-increasing time_from_start
		trajectory.joint_trajectory.points.pop_back();
		
		// Time parameterization to ensure scarfer's trajectory is realistically and adequately slow
		robot_trajectory::RobotTrajectory rt(move_group_interface->getCurrentState()->getRobotModel(), planning_grp);
		rt.setRobotTrajectoryMsg(*move_group_interface->getCurrentState(), trajectory);
		trajectory_processing::IterativeParabolicTimeParameterization iptp;
		iptp.computeTimeStamps(rt, 0.005, 0.1);
		rt.getRobotTrajectoryMsg(trajectory);


		
		// Execute region coverage path plan asynchronously
		(*move_group_interface).asyncExecute(trajectory);
		
		// Prevent setting cartesian_start before arm begins moving
		int counter = 0;
		while (move_wall != true || counter < 3){
			counter += 1;
			ros::Duration(1.0).sleep();
		}
		cartesian_start = true;
		is_scarf_execute_bool.data = cartesian_start;
		is_scarf_execute_pub.publish(is_scarf_execute_bool);
	}
	
	// Set target pose for moving end effector to
	void set_goal(const geometry_msgs::Pose& pose){
		target_pose.position.x = pose.position.x;
		target_pose.position.y = pose.position.y;
		target_pose.position.z = pose.position.z;
		target_pose.orientation.x = pose.orientation.x;
		target_pose.orientation.y = pose.orientation.y;
		target_pose.orientation.z = pose.orientation.z;
		target_pose.orientation.w = pose.orientation.w;
	}
	
	// Move end effector to target pose (non-cartesian)
	void move_to_goal(){
		(*move_group_interface).setPoseTarget(target_pose);
		moveit::planning_interface::MoveGroupInterface::Plan move_plan;
		if ((*move_group_interface).plan(move_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS){
			if ((*move_group_interface).execute(move_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS){
				robot_state::RobotStatePtr end_state_reached = (*move_group_interface).getCurrentState();
				const Eigen::Isometry3d& end_effector_state = end_state_reached->getGlobalLinkTransform("arm2_wrist_3_link");
				ROS_INFO_STREAM("Translation: \n" << end_effector_state.translation() << "\n");
				ROS_INFO_STREAM("Rotation: \n" << end_effector_state.rotation() << "\n");
			}
		}
	}
	
	// Finds the position of the link that is closest to the evaluator
	std::string find_protruding(){
		std::vector<std::string> links = (*move_group_interface).getLinkNames();
		robot_state::RobotStatePtr end_state_reached = (*move_group_interface).getCurrentState();
		
		const Eigen::Isometry3d& link_state = end_state_reached->getGlobalLinkTransform(links[0]);
		// Effector is in negative x direction from viz
		float closest_link_pose = (link_state.translation())(0);
		std::string closest_link = links[0];
		for (int i = 1; i < links.size(); i++){
			const Eigen::Isometry3d& link_state = end_state_reached->getGlobalLinkTransform(links[i]);
			if (closest_link_pose < (link_state.translation())(0)){
				closest_link_pose = (link_state.translation())(0);
				closest_link = links[i];
			}
		}
		//ROS_INFO_STREAM("closest: \n" << closest_link << "\n");
		return closest_link;
	}
	
	// Creates a wall to segment of the existing region that the scarfer is working on
	void wall_off(){
		// if manipulator is in motion and in region of coverage path plan, begin to segment workspace
		if (move_wall && cartesian_start){
			std::string protrude_link = find_protruding();
			robot_state::RobotStatePtr end_state_reached = (*move_group_interface).getCurrentState();
			const Eigen::Isometry3d& protrude_link_iso = end_state_reached->getGlobalLinkTransform(protrude_link);
			
			geometry_msgs::Pose wall_pose;
			wall_pose.position.x = (protrude_link_iso.translation())(0) + 0.2;
			wall_pose.position.y = (protrude_link_iso.translation())(1);
			wall_pose.position.z = (protrude_link_iso.translation())(2);
			wall_pose.orientation.x = 0.0;
			wall_pose.orientation.y = 0.7071068;
			wall_pose.orientation.z = 0.0;
			wall_pose.orientation.w = 0.7071068;
			wall_ing(wall_pose, 3.0, 3.0);
		// if manipulator is idling and in region of coverage path plan, clean up walls from planning scene
		}else if (cartesian_start && not move_wall){
			cartesian_start = false;
			is_scarf_execute_bool.data = cartesian_start;
			is_scarf_execute_pub.publish(is_scarf_execute_bool);
			if (collision_objects.size() != 1){
				collision_objects.pop_back();
			}
			planning_scene_interface->addCollisionObjects(collision_objects);
		}
	}
	
	void wall_ing(const geometry_msgs::Pose& pose, float length, float width){
		moveit_msgs::CollisionObject collision_object;
		// Unused pose field in moveit_msgs::CollisionObject, set to remove warning
		collision_object.pose.orientation.w = 1.0;
		collision_object.pose.position.x = 0.0;
		
		collision_object.header.frame_id = (*move_group_interface).getPlanningFrame();
		
		collision_object.id = "wall";
		shape_msgs::SolidPrimitive primitive;
		primitive.type = primitive.BOX;
		primitive.dimensions.resize(3);
		primitive.dimensions[primitive.BOX_X] = length;
		primitive.dimensions[primitive.BOX_Y] = width;
		primitive.dimensions[primitive.BOX_Z] = 0.01;
		
		geometry_msgs::Pose floor_pose;
		floor_pose.orientation.x = pose.orientation.x;
		floor_pose.orientation.y = pose.orientation.y;
		floor_pose.orientation.z = pose.orientation.z;
		floor_pose.orientation.w = pose.orientation.w;
		floor_pose.position.x = pose.position.x;
		floor_pose.position.y = pose.position.y;
		floor_pose.position.z = pose.position.z;

		collision_object.primitives.push_back(primitive);
		collision_object.primitive_poses.push_back(floor_pose);
		collision_object.operation = collision_object.ADD;
		
		if (collision_objects.size() != 1){
			collision_objects.pop_back();
		}
		collision_objects.push_back(collision_object);
		
		planning_scene_interface->addCollisionObjects(collision_objects);
	}
};


namespace rvt = rviz_visual_tools;


int main(int argc, char** argv)
{
	// Node setup
	ros::init(argc, argv, "one_arm_move_eff");
	ros::AsyncSpinner spinner(1);
	spinner.start();
	
	float scarf_center[7] = {0.5, 0, 0.5, 0, 0.7071068, 0, 0.7071068};
	
	std::string grp = "ur5_arm2"; // ur5_arm2 -> eff, ur5_arm1 -> viz
	moveit::planning_interface::MoveGroupInterface move0(grp);
	moveit::planning_interface::PlanningSceneInterface scene0;

	scarf scarfer(grp, &move0, &scene0);
	
	
	while(ros::ok()){
		scarfer.wall_off();
		
		ros::spinOnce();
	}
	return 0;
}
