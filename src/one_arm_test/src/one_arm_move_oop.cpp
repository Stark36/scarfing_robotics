#include <iostream>
#include <cmath>
#include <vector>

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
#include <moveit_msgs/OrientationConstraint.h>

using namespace std;

struct Point {
	float x;
	float y;
};

struct Poly_bound_box{
	float init_vert;
	float box[4]; // {max_x, min_x, max_y, min_y}
};

class scarf{

	private:
	float head_diameter;
	float scarf_diameter;
	float scarf_center[7];
	int resolution;
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
	ros::Publisher path_pub;
	
	scarf(std::string grp, moveit::planning_interface::MoveGroupInterface *move0, moveit::planning_interface::PlanningSceneInterface *scene0){
		count = 0;
		path.header.frame_id="map";

		temp_sub = node_handle.subscribe("/temp",1, &scarf::temp, this);
		path_pub = node_handle.advertise<nav_msgs::Path>("/get_path",1);
		
		planning_grp = grp;
		move_group_interface = move0;
		planning_scene_interface = scene0;
		joint_model_group = (*move_group_interface).getCurrentState()->getJointModelGroup(planning_grp);
	}

	void temp(const std_msgs::Int64& msg){
		ROS_INFO_STREAM("Temp: \n" << msg.data << "\n");
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
				const Eigen::Isometry3d& end_effector_state = end_state_reached->getGlobalLinkTransform("wrist_3_link");
				ROS_INFO_STREAM("Translation: \n" << end_effector_state.translation() << "\n");
				ROS_INFO_STREAM("Rotation: \n" << end_effector_state.rotation() << "\n");
			}
		}
	}
	
	void set_scarf_parameters(float head_dia, float scarf_dia, float center[7], int res){
		head_diameter = head_dia;
		scarf_diameter = scarf_dia;
		
		for (int i = 0; i < 7; ++i) {
			scarf_center[i] = center[i];
    		}
		resolution = res;
	}
	
	void create_circ_path_plan(){
		int panes =floor((scarf_diameter-head_diameter)/head_diameter*2);
		float diameter = 0.0;
		int segments = 0;
		float step = 0;	
		int counter = 0;

		for (int i = 0; i < panes; i++){
			segments = resolution*floor(1+diameter/head_diameter*2);
			diameter += head_diameter/2;
			step = 2*M_PI/segments;
			for (int j = 0; j < segments; j++){
		    		geometry_msgs::PoseStamped post;
		    		post.header.seq = counter;
				counter += 1;
		    		post.header.stamp = ros::Time::now();
		    		post.header.frame_id = "map";
		    		
		    		post.pose.position.x = diameter/2.0*sin(j*step) + scarf_center[0];
				post.pose.position.y = diameter/2.0*cos(j*step) + scarf_center[1];
				post.pose.position.z = scarf_center[2];
				path.poses.push_back(post);
			}
		}
		path_pub.publish(path);
	}
	
	
	
	Poly_bound_box bounding(const vector<Point> &vertices){
	    int n_vertices = vertices.size();

	    Poly_bound_box bounded;
	    float min_area = 0;
	    
	    Point pt1;
	    Point pt2;
	    float rad;
	    Point temp_pt;
	    float area;
	    for (int i = 0; i < n_vertices; i++){
		pt1 = vertices[i];
		pt2 = vertices[(i+1)%n_vertices];
		rad = -atan((pt2.y-pt1.y)/(pt2.x-pt1.x));
		
		float max_x, max_y, min_x, min_y;
		max_x = min_x = cos(rad)*vertices[0].x-sin(rad)*vertices[0].y;
		max_y = min_y = sin(rad)*vertices[0].x+cos(rad)*vertices[0].y;
		for (int j = 1; j < n_vertices; j++){
		    temp_pt.x = cos(rad)*vertices[j].x-sin(rad)*vertices[j].y;
		    temp_pt.y = sin(rad)*vertices[j].x+cos(rad)*vertices[j].y;
		    if (temp_pt.x > max_x){
		        max_x = temp_pt.x;
		    }else if (temp_pt.x < min_x){
		        min_x = temp_pt.x;
		    }
		    if (temp_pt.y>max_y){
		        max_y = temp_pt.y;
		    }else if (temp_pt.y<min_y){
		        min_y = temp_pt.y;
		    }
		}
		area = (max_y-min_y)*(max_x-min_x);
		if (area<min_area or i == 0){
		    min_area = area;
		    bounded.init_vert = i;
		    bounded.box[0] = max_x;
		    bounded.box[1] = min_x;
		    bounded.box[2] = max_y;
		    bounded.box[3] = min_y;
		}
	    }
	    ROS_INFO_STREAM("Minimum area of bounding box: " << min_area << "\n");
	    return bounded;
	}
		
	float sub_line_pt(Point pt1, Point pt2, Point q_pt) {
	    return ((q_pt.y-pt1.y)*(pt2.x-pt1.x) - (q_pt.x-pt1.x) *(pt2.y-pt1.y));
	}
	
	int inpoly(const Point &q_pt, const vector<Point> &vertices) {
	    float tol = 1e-6;
	    int wn = 0;  // the  winding number counter
	    const int n_vertices = vertices.size();

	    for (int i = 0; i < n_vertices; i++) {
		float point_in_line = sub_line_pt(vertices[i], vertices[(i+1)%n_vertices], q_pt);
		// Lies on the polygon might not exactly be equal to zero due to finite precision

		if (abs(point_in_line) < tol) {
		    if (((q_pt.x-vertices[i].x)/(vertices[(i+1)%n_vertices].x-vertices[i].x)>(1+tol)) || ((q_pt.x-vertices[i].x)/(vertices[(i+ 1)%n_vertices].x-vertices[i].x)<(0-tol)) || ((q_pt.y-vertices[i].y)/(vertices[(i+1)%n_vertices].y-vertices[i].y)>(1+tol)) || ((q_pt.y-vertices[i].y)/(vertices[(i+ 1)%n_vertices].y-vertices[i].y)<(0-tol))){
		        return 0;
		    }else{
		        return 1;
		    }
		}
		// Upwards crossing
		if (point_in_line > 0){
		    if((vertices[i].y <= q_pt.y) && (vertices[(i+1)%n_vertices].y>q_pt.y)){
		        wn++;
		    }
		}
		// Downwards crossing
		else{
		    if((vertices[i].y > q_pt.y) && (vertices[(i+1)%n_vertices].y<=q_pt.y)){
		        wn--;
		    }
		}
	    }
	    // Point is inside polygon only if wn != 0
	    return (wn != 0);  
	}
	
	
	// Given three collinear points p, q, r, the function checks if point q lies on line segment 'pr'
	bool onSegment(Point p, Point q, Point r){
	    if (q.x <= max(p.x, r.x) && q.x >= min(p.x, r.x) && q.y <= max(p.y, r.y) && q.y >= min(p.y, r.y)){
	    	return true;
	    }
	    return false;
	}
  
	// To find orientation of ordered triplet (p, q, r).
	// The function returns following values
	// 0 --> p, q and r are collinear
	// 1 --> Clockwise
	// 2 --> Counterclockwise
	int orientation(Point p, Point q, Point r){
	    int val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);
	    if (val == 0) return 0;  // collinear
	    return (val > 0)? 1: 2; // clock or counterclock wise
	}
  
	// The main function that returns true if line segment 'p1q1' and 'p2q2' intersect.
	bool doIntersect(Point p1, Point q1, Point p2, Point q2){
	    // Find the four orientations needed for general and special cases
	    int o1 = orientation(p1, q1, p2);
	    int o2 = orientation(p1, q1, q2);
	    int o3 = orientation(p2, q2, p1);
	    int o4 = orientation(p2, q2, q1);
	  
	    // General case
	    if (o1 != o2 && o3 != o4){
		return true;
	    }
	    // Special Cases
	    // p1, q1 and p2 are collinear and p2 lies on segment p1q1
	    if (o1 == 0 && onSegment(p1, p2, q1)){
		return true;
	    }
	    // p1, q1 and q2 are collinear and q2 lies on segment p1q1
	    if (o2 == 0 && onSegment(p1, q2, q1)){
		return true;
	    }
	    // p2, q2 and p1 are collinear and p1 lies on segment p2q2
	    if (o3 == 0 && onSegment(p2, p1, q2)){
	    	return true;
	    }
	     // p2, q2 and q1 are collinear and q1 lies on segment p2q2
	    if (o4 == 0 && onSegment(p2, q1, q2)){
	    	return true;
	    }
	    return false; // Doesn't fall in any of the above cases
	}
	

	
	void create_poly_path_plan(const vector<vector<float>> &vertices){
	    int n_vertices = vertices.size();
	    int counter  = 0;
	    
	    if (n_vertices >= 3){
		vector<Point> pt_vertices;
		for (int i = 0; i < n_vertices; i++){
		    Point temp;
		    temp.x = vertices[i][0];
		    temp.y = vertices[i][1];
		    pt_vertices.push_back(temp);
		}
		Poly_bound_box bounded;
		bounded = bounding(pt_vertices);
		
		//reorder points
		for (int i = 0; i < bounded.init_vert; i++){
		    Point temp;
		    temp = pt_vertices[0];
		    pt_vertices.erase(pt_vertices.begin());
		    pt_vertices.push_back(temp);
		}
		

		// direction of sweep
		vector<float> dir = {(pt_vertices[1].x-pt_vertices[0].x), (pt_vertices[1].y-pt_vertices[0].y)};
		ROS_INFO_STREAM("dir: " <<  dir[0] << ", " << dir[1] << "\n");
		float mag = sqrt(pow(dir[0],2) + pow(dir[1],2));
		dir[0] /=mag;
		dir[1] /=mag;
		vector<float> sweep_dir = {-dir[1], dir[0]};
		
		float res = 0.001;
		float swath = 0.005;
		float antipodal_dist = bounded.box[2]-bounded.box[3];
		float dist = 0;
		Point prev_pt;
		float alpha;
		
		ROS_INFO_STREAM("Antipodal distance: " << antipodal_dist << "\n");
		Point move_pt = pt_vertices[0];
		while (true){
		    ROS_INFO_STREAM("Distance: " <<  dist << "\n");
		    while (inpoly(move_pt, pt_vertices)){
		    	geometry_msgs::PoseStamped post;
		    	post.header.seq = counter;
			counter += 1;
		    	post.header.stamp = ros::Time::now();
		    	post.header.frame_id = "map";
		    		
		    	post.pose.position.x = move_pt.x;
			post.pose.position.y = move_pt.y;
			post.pose.position.z = 0.5;
			path.poses.push_back(post);
		        
		        prev_pt = move_pt;
		        move_pt.x += res/sqrt(2)*dir[0];
		        move_pt.y += res/sqrt(2)*dir[1];
        		path_pub.publish(path);
		    }
    		    ROS_INFO_STREAM("Prev pt: " << prev_pt.x << ", " << prev_pt.y << "\n");
		    ROS_INFO_STREAM("Move pt: " << move_pt.x << ", " << move_pt.y << "\n");

    		    path_pub.publish(path);
    		    ROS_INFO_STREAM("Sweeping\n");
		    
		    for (int i = 1; i < n_vertices; i++) {
		    	if(doIntersect(prev_pt, move_pt, pt_vertices[i], pt_vertices[(i+ 1)%n_vertices])){
    		    		ROS_INFO_STREAM("Found intersection\n");
		    		alpha = ((pt_vertices[(i+ 1)%n_vertices].y-pt_vertices[i].y)*(prev_pt.x-pt_vertices[i].x)-(pt_vertices[(i+ 1)%n_vertices].x-pt_vertices[i].x)*(prev_pt.y-pt_vertices[i].y))/((pt_vertices[(i+ 1)%n_vertices].x-pt_vertices[i].x)*dir[1]-(pt_vertices[(i+ 1)%n_vertices].y-pt_vertices[i].y)*dir[0]);
				
				move_pt.x = prev_pt.x+alpha*dir[0];
				move_pt.y = prev_pt.y+alpha*dir[1];
				
				geometry_msgs::PoseStamped post;
			    	post.header.seq = counter;
				counter += 1;
			    	post.header.stamp = ros::Time::now();
			    	post.header.frame_id = "map";
			    		
			    	post.pose.position.x = move_pt.x;
				post.pose.position.y = move_pt.y;
				post.pose.position.z = 0.5;
				path.poses.push_back(post);

		    	}
		    }
		    
		    path_pub.publish(path);
		    ROS_INFO_STREAM("Advancing\n");
		    prev_pt = move_pt;
		    if (dist >= antipodal_dist){
		    	break;
		    }else if (antipodal_dist - dist >= (swath+1e-6)){
		    	dist += swath;
		    	move_pt.x += swath*sweep_dir[0];
		    	move_pt.y += swath*sweep_dir[1];
		    }else if (antipodal_dist - dist < (swath+1e-6)){
		    	dist = antipodal_dist;
		    	move_pt.x += (antipodal_dist - dist)*sweep_dir[0];
		    	move_pt.y += (antipodal_dist - dist)*sweep_dir[1];
		    }else{
		    	ROS_INFO_STREAM("Forgotten scenario.\n");
		    }

		    dir[0] = -dir[0];
		    dir[1] = -dir[1];
		    
		    ROS_INFO_STREAM("Prev pt: " << prev_pt.x << ", " << prev_pt.y << "\n");
		    ROS_INFO_STREAM("Move pt: " << move_pt.x << ", " << move_pt.y << "\n");
		    
		    if (!inpoly(move_pt, pt_vertices)){
		    	ROS_INFO_STREAM("Folding in\n");
		    	while (!inpoly(move_pt, pt_vertices)){
				prev_pt = move_pt;			
				move_pt.x += res/sqrt(2)*dir[0];
				move_pt.y += res/sqrt(2)*dir[1];
		    	}
		    	ROS_INFO_STREAM("Folded\n");
		    	for (int i = 1; i < n_vertices; i++) {
			    	if(doIntersect(prev_pt, move_pt, pt_vertices[i], pt_vertices[(i+ 1)%n_vertices])){
					ROS_INFO_STREAM("Found intersection\n");
					alpha = ((pt_vertices[(i+ 1)%n_vertices].y-pt_vertices[i].y)*(prev_pt.x-pt_vertices[i].x)-(pt_vertices[(i+ 1)%n_vertices].x-pt_vertices[i].x)*(prev_pt.y-pt_vertices[i].y))/((pt_vertices[(i+ 1)%n_vertices].x-pt_vertices[i].x)*dir[1]-(pt_vertices[(i+ 1)%n_vertices].y-pt_vertices[i].y)*dir[0]);
					
					prev_pt = move_pt;
					move_pt.x = prev_pt.x+alpha*dir[0];
					move_pt.y = prev_pt.y+alpha*dir[1];
			    	}
		    	}
		    }else{
		    	ROS_INFO_STREAM("Folding out\n");
		    	while (inpoly(move_pt,pt_vertices)){
				prev_pt = move_pt;			
				move_pt.x -= res/sqrt(2)*dir[0];
				move_pt.y -= res/sqrt(2)*dir[1];
		    	}
		    	ROS_INFO_STREAM("Folded\n");
		    	for (int i = 1; i < n_vertices; i++) {
			    	if(doIntersect(prev_pt, move_pt, pt_vertices[i], pt_vertices[(i+ 1)%n_vertices])){
					ROS_INFO_STREAM("Found intersection\n");
					alpha = ((pt_vertices[(i+ 1)%n_vertices].y-pt_vertices[i].y)*(move_pt.x-pt_vertices[i].x)-(pt_vertices[(i+ 1)%n_vertices].x-pt_vertices[i].x)*(move_pt.y-pt_vertices[i].y))/((pt_vertices[(i+ 1)%n_vertices].x-pt_vertices[i].x)*-dir[1]-(pt_vertices[(i+ 1)%n_vertices].y-pt_vertices[i].y)*-dir[0]);
					 
					prev_pt = move_pt;
					move_pt.x -= alpha*dir[0];
					move_pt.y -= alpha*dir[1];
			    	}
		    	}
		    }
		}
		path_pub.publish(path);
		ROS_INFO_STREAM("Path planned\n");
	    }
	}
};


namespace rvt = rviz_visual_tools;


int main(int argc, char** argv)
{
	// Node setup
	ros::init(argc, argv, "one_arm_move");
	ros::AsyncSpinner spinner(1);
	spinner.start();
	
	// Circle path plan setup parameters
	float head_diameter = 0.01;
	float scarf_diameter = 0.2;
	float scarf_center[7] = {0.5, 0, 0.5, 0, 0.7071068, 0, 0.7071068};
	int resolution = 36;
	
	// Polygon path plan setup parameters
	vector<vector<float>> vertices = {{0.5, 0.5}, {0.59, 0.6}, {0.6, 0.72}, {0.45, 0.7}, {0.45, 0.55}};
	
	std::string grp = "ur5_arm";
	moveit::planning_interface::MoveGroupInterface move0(grp);
	moveit::planning_interface::PlanningSceneInterface scene0;
	
	scarf scarfer(grp, &move0, &scene0);
	ros::Duration(1.0).sleep();
	//scarfer.set_scarf_parameters(head_diameter, scarf_diameter, scarf_center, resolution);
	//scarfer.create_circ_path_plan();
	scarfer.create_poly_path_plan(vertices);
	
	geometry_msgs::Pose temp_pose;
	temp_pose.position.x = scarf_center[0];
	temp_pose.position.y = scarf_center[1];
	temp_pose.position.z = scarf_center[2];
	temp_pose.orientation.x = scarf_center[3];
	temp_pose.orientation.y = scarf_center[4];
	temp_pose.orientation.z = scarf_center[5];
	temp_pose.orientation.w = scarf_center[6];
	
	
	scarfer.set_goal(temp_pose);
	scarfer.move_to_goal();
	
	while(ros::ok()){
		ros::spinOnce();
	}
	return 0;
}
