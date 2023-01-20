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

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <nav_msgs/Path.h>
#include <std_msgs/Int64.h>
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
	geometry_msgs::Pose target_pose;
	nav_msgs::Path path;
	
	std::string planning_grp;
	moveit::planning_interface::MoveGroupInterface *move_group_interface;
	moveit::planning_interface::PlanningSceneInterface *planning_scene_interface;
	const moveit::core::JointModelGroup *joint_model_group;
	
	public:
	ros::NodeHandle node_handle;
	ros::Subscriber temp_sub;
	ros::Subscriber path_sub;
	
	scarf(std::string grp, moveit::planning_interface::MoveGroupInterface *move0, moveit::planning_interface::PlanningSceneInterface *scene0){
		count = 0;
		path.header.frame_id="world";
		
		temp_sub = node_handle.subscribe("/temp",1, &scarf::temp, this);
		path_sub = node_handle.subscribe("/get_path",1, &scarf::path_callback, this);
				
		planning_grp = grp;
		move_group_interface = move0;
		(*move_group_interface).setPlanningTime(60.0);
		planning_scene_interface = scene0;
		joint_model_group = (*move_group_interface).getCurrentState()->getJointModelGroup(planning_grp);
	}

	void temp(const std_msgs::Int64& msg){
		ROS_INFO_STREAM("Temp: " << msg.data << "\n");
	}
	
	void path_callback(const nav_msgs::Path& input){
		std::vector<geometry_msgs::Pose> waypoints;
	
		path.poses = input.poses;
		int n_waypoints = path.poses.size();
		ROS_INFO_STREAM("Number of waypoints: " << n_waypoints << "\n");
		path.poses[0].pose.orientation.x = 0.0;
		path.poses[0].pose.orientation.y = 0.7071068;
		path.poses[0].pose.orientation.z = 0.0;
		path.poses[0].pose.orientation.w = 0.7071068;
		path.poses[0].pose.position.z += 0.2;
		set_goal(path.poses[0].pose);
		move_to_goal();
		
		for (int i = 1; i < n_waypoints; i++){
			path.poses[i].pose.orientation.x = 0.0;
			path.poses[i].pose.orientation.y = 0.7071068;
			path.poses[i].pose.orientation.z = 0.0;
			path.poses[i].pose.orientation.w = 0.7071068;
			path.poses[i].pose.position.z += 0.2;
			waypoints.push_back(path.poses[i].pose);
		}
		moveit_msgs::RobotTrajectory trajectory;
		const double jump_threshold = 0.0;
		const double eef_step = 0.01;
		double fraction = (*move_group_interface).computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
		
		if ((*move_group_interface).execute(trajectory) == moveit::planning_interface::MoveItErrorCode::SUCCESS){
			ROS_INFO_STREAM("Completed" <<"\n");
		}
		segment();
	}
	
	void set_goal(const geometry_msgs::Pose& pose){
		target_pose.position.x = pose.position.x;
		target_pose.position.y = pose.position.y;
		target_pose.position.z = pose.position.z;
		target_pose.orientation.x = pose.orientation.x;
		target_pose.orientation.y = pose.orientation.y;
		target_pose.orientation.z = pose.orientation.z;
		target_pose.orientation.w = pose.orientation.w;
	}
	
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
	
	void segment(){
		std::vector<std::string> links = (*move_group_interface).getLinkNames();
		robot_state::RobotStatePtr end_state_reached = (*move_group_interface).getCurrentState();
		
		const Eigen::Isometry3d& link_state = end_state_reached->getGlobalLinkTransform(links[0]);
		float closest_link = (link_state.translation())(0); // eff is in negative x direction from viz
		for (int i = 1; i < links.size(); i++){
			const Eigen::Isometry3d& link_state = end_state_reached->getGlobalLinkTransform(links[i]);
			if (closest_link < (link_state.translation())(0)){
				closest_link = (link_state.translation())(0);
			}
		}
		ROS_INFO_STREAM("closest: \n" << closest_link << "\n");
	}
	
	void wall_off(const geometry_msgs::Pose& pose, float length, float width){
		moveit_msgs::CollisionObject collision_object;
		collision_object.header.frame_id = (*move_group_interface).getPlanningFrame();
		
		collision_object.id = "wall";
		shape_msgs::SolidPrimitive primitive;
		primitive.type = primitive.BOX;
		primitive.dimensions.resize(3);
		primitive.dimensions[primitive.BOX_X] = length;
		primitive.dimensions[primitive.BOX_Y] = width;
		primitive.dimensions[primitive.BOX_Z] = 0.05;
		
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

		std::vector<moveit_msgs::CollisionObject> collision_objects;
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
		ros::spinOnce();
	}
	return 0;
}
