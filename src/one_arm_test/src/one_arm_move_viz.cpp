#include <iostream>
#include <cmath>
#include <vector>
#include <algorithm>

#include <ros/ros.h>
#include <tf/transform_listener.h>

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
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point32.h>
#include <moveit_msgs/OrientationConstraint.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <geometry_msgs/Point32.h>

using namespace std;

struct Point {
	float x;
	float y;
};

// structure for minimum bounding box
struct Poly_bound_box{
	float init_vert;
	float box[4]; // {max_x, min_x, max_y, min_y}
};


struct kd_node{
    geometry_msgs::Point32 data;
    int axis;
    kd_node *l_child = NULL;
    kd_node *r_child = NULL;
    kd_node *parent = NULL;
};

// Helper function for use in nth_element in median finding (partial sorting)
// Comparing x dimension of a point (x, y, z)
bool dim_comp_0(geometry_msgs::Point32 a, geometry_msgs::Point32 b){
	return (a.x < b.x);
}

// Comparing y dimension of a point (x, y, z)
bool dim_comp_1(geometry_msgs::Point32 a, geometry_msgs::Point32 b){
	return (a.y < b.y);
}

// Use of cpp's partial sorting algorithm for median finding 
int findMedian(geometry_msgs::Point32 a[], int dim, int start, int end){
	int med = (end-start+1)/2+start;
	if (dim == 0){
		nth_element(a+start, a+med, a+end+1, dim_comp_0);
	}else{
		nth_element(a+start, a+med, a+end+1, dim_comp_1);
	}
	return med;
}

// kdtree class
class kdtree{
	public:
	kd_node* root;
	kd_node* temp;
	    	
	kd_node* construct(geometry_msgs::Point32 cloud[], kd_node *par, int dim, int start, int end){
		kd_node* temp_node;
		temp_node = new kd_node;
		int median;
		if (start == end){
			temp_node->data = cloud[start];
			temp_node->axis = dim;
			temp_node->parent = par;
		}else if (start>end){
			return NULL;
		}else{
			median = findMedian(cloud, dim, start, end);
			temp_node->data = cloud[median];
			temp_node->axis = dim;
			temp_node->parent = par;
			temp_node->l_child = construct(cloud, temp_node, (dim+1)%2, start, median-1);
			temp_node->r_child = construct(cloud, temp_node, (dim+1)%2, median+1, end);
		}
		//cout<<"start: "<<start<<" end: "<<end<<endl;
		//cout<<temp_node->data.x<<", "<<temp_node->data.y<<endl;
		return temp_node;
	}

    	// Find euclidean distance between 2 points projected in a plan (x-y plane)
	float find_dst(geometry_msgs::Point32 a, geometry_msgs::Point32 b){
		return sqrt(pow((b.x-a.x), 2) + pow((b.y-a.y), 2));
	}
    	
    	// Find distance between 2 points in a dimension (x or y axis)
	float find_dst(geometry_msgs::Point32 a, geometry_msgs::Point32 b, int dim){
		if (dim==0){
			return abs((b.x-a.x));
		}else{
			return abs((b.y-a.y));
		}
	}
    
    	// Determine whether node is right (1) or left(0) child of parent node
	int which_child(kd_node *q_n){
		if (q_n->parent->l_child == q_n){
			return 0;
		}else{
			return 1;
		}
	}
    
	// Recursively travels down tree to find the leaf node closest to point queried(q_pt)
	kd_node* find_nearest_leaf(geometry_msgs::Point32 q_pt, int dim, kd_node *s_pt){
		//kd_node* temp;
		if (dim == 0){
			if (q_pt.x >= s_pt->data.x){
				if (s_pt->r_child == NULL){
					return s_pt;
				}else{
					temp = find_nearest_leaf(q_pt, (dim+1)%2, s_pt->r_child);
					return temp;
				}

			}else{
				if (s_pt->l_child == NULL){
					return s_pt;
				}else{
					temp = find_nearest_leaf(q_pt, (dim+1)%2, s_pt->l_child);
					return temp;
				}

			}
		}else{
			if (q_pt.y >= s_pt->data.y){
				if (s_pt->r_child == NULL){
					return s_pt;
				}else{
					temp = find_nearest_leaf(q_pt, (dim+1)%2, s_pt->r_child);
					return temp;
				}

			}else{
				if (s_pt->l_child == NULL){
					return s_pt;
				}else{
					temp = find_nearest_leaf(q_pt, (dim+1)%2, s_pt->l_child);
					return temp;
				}
			}
		}
	}
    
    	// Recursively travels up from leaf node to find the node closest to point queried(q_pt)
	kd_node* find_nearest(geometry_msgs::Point32 q_pt, int dim, kd_node *s_pt){
		kd_node *nnode, *par_n, *tem_n;
		float min_dist, dist;
		int ax;
		nnode = find_nearest_leaf(q_pt, dim, s_pt);
		min_dist = find_dst(q_pt, nnode->data);
		tem_n = nnode;
		par_n = tem_n->parent;
        
		while (par_n != NULL){
			ax = tem_n->axis;
			dist = find_dst(q_pt, tem_n->data, ax);
			//cout<< "min dist " << min_dist <<endl;
			//cout<< "axis dist "<< dist << endl;
			//cout<< tem_n->data.x << ", " << tem_n->data.y << endl;
			if (dist < min_dist){
				dist = find_dst(q_pt, tem_n->data);
				if (dist<min_dist){
					min_dist = dist;
					nnode = tem_n;
				}
				if (which_child(tem_n)==0 && par_n->r_child != NULL){
					tem_n = find_nearest_leaf(q_pt, ax, par_n->r_child);
					dist = find_dst(q_pt, tem_n->data);
					if (dist<min_dist){
						min_dist = dist;
						nnode = tem_n;
					}
                		}else if (which_child(tem_n)==1 && par_n->l_child != NULL){
					tem_n = find_nearest_leaf(q_pt, ax, par_n->l_child);
					dist = find_dst(q_pt, tem_n->data);
					if (dist<min_dist){
						min_dist = dist;
						nnode = tem_n;
					}
				}

			}
			//cout<< "cal dist "<< dist << endl;
			//cout<< tem_n->data.x << ", " << tem_n->data.y << endl;
			tem_n = par_n;
			par_n = tem_n->parent;
		}
		return nnode;
	}
};

// Evaluator class
class eval{

	private:
	int count;
	kdtree tree;
	sensor_msgs::PointCloud cloud_scan;
	
	geometry_msgs::Pose target_pose;
	nav_msgs::Path path;
	
	std::string arm1_planning_grp;
	std::string arm2_planning_grp;
	moveit::planning_interface::MoveGroupInterface *arm1_move_group_interface;
	moveit::planning_interface::MoveGroupInterface *arm2_move_group_interface;
	moveit::planning_interface::PlanningSceneInterface *planning_scene_interface;
	const moveit::core::JointModelGroup *arm1_joint_model_group;
	const moveit::core::JointModelGroup *arm2_joint_model_group;
	
	bool is_scarf_execute = false;
	
	public:
	int kd_tree_maker = 1;
	
	ros::NodeHandle node_handle;
	ros::Subscriber temp_sub;
	ros::Subscriber pcl_sub;
	ros::Subscriber is_scarf_execute_sub;
	ros::Publisher path_pub;
	ros::Publisher arm1_command_pub;
	tf::TransformListener listener;
	
	eval(std::string grp, moveit::planning_interface::MoveGroupInterface *move0, moveit::planning_interface::PlanningSceneInterface *scene0, moveit::planning_interface::MoveGroupInterface *move1){
		count = 0;
		path.header.frame_id="world";
		
		// Set up of subscribers and publishers
		temp_sub = node_handle.subscribe("/temp",1, &eval::temp, this);
		pcl_sub = node_handle.subscribe("/pcl/points",1, &eval::pcl_callback, this);
		is_scarf_execute_sub = node_handle.subscribe("/is_scarf_execute",1, &eval::is_scarf_execute_callback, this);
		path_pub = node_handle.advertise<nav_msgs::Path>("/get_path",1);
		arm1_command_pub = node_handle.advertise<trajectory_msgs::JointTrajectory>("/ur5_arm1_controller/command",0);

		// Linking to moveit group and planning scene interface
		arm1_planning_grp = grp;
		arm1_move_group_interface = move0;
		(*arm1_move_group_interface).setPlanningTime(10.0);
		planning_scene_interface = scene0;
		arm1_joint_model_group = (*arm1_move_group_interface).getCurrentState()->getJointModelGroup(arm1_planning_grp);
		
		arm2_move_group_interface = move1;
	}

	void temp(const std_msgs::Int64& msg){
		ROS_INFO_STREAM("Temp: \n" << msg.data << "\n");
	}
	
	// listens to pointcloud being published and constructs a kd_tree from pointcloud once evaluator is in position
	void pcl_callback(const sensor_msgs::PointCloud2ConstPtr& input){
		if (kd_tree_maker == 0){
			sensor_msgs::convertPointCloud2ToPointCloud(*input, cloud_scan);
			ROS_INFO_STREAM("Number of pts: " << cloud_scan.points.size() << "\n");
			int n = cloud_scan.points.size();
			geometry_msgs::Point32* cld_pts = &cloud_scan.points[0];
			tree.root = tree.construct(cld_pts, NULL, 0, 0, n-1);
			
			kd_node *temp_node;
			geometry_msgs::Point32 temp_pt;
		    	temp_pt.x = 0.05;
		    	temp_pt.y = 0.05;
			temp_node = tree.find_nearest(temp_pt, 0, tree.root);
			
			ROS_INFO_STREAM("root: " << tree.root->data.z  << "\n");
			kd_tree_maker = 1;
		}
		
	}
	
	// Checks if effector has begun region of path plan
	void is_scarf_execute_callback(std_msgs::Bool msg){
		is_scarf_execute = msg.data;
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
		(*arm1_move_group_interface).setPoseTarget(target_pose);
		moveit::planning_interface::MoveGroupInterface::Plan move_plan;
		if ((*arm1_move_group_interface).plan(move_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS){
			if ((*arm1_move_group_interface).execute(move_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS){
				robot_state::RobotStatePtr end_state_reached = (*arm1_move_group_interface).getCurrentState();
				const Eigen::Isometry3d& end_effector_state = end_state_reached->getGlobalLinkTransform("arm1_wrist_3_link");
				ROS_INFO_STREAM("Translation: \n" << end_effector_state.translation() << "\n");
				ROS_INFO_STREAM("Rotation: \n" << end_effector_state.rotation() << "\n");
			}
		}
	}
	
	// Creates a radial path plan (2D)
	void create_circ_path_plan(float head_diameter, float scarf_diameter, float scarf_center[7], int resolution){
		int panes =floor((scarf_diameter-head_diameter)/head_diameter*2);
		float radius = 0.0;
		int segments = 0;
		float step = 0;	
		int counter = 0;

		for (int i = 0; i < panes; i++){
			segments = resolution*floor(1+radius/head_diameter*2);
			radius += head_diameter/2;
			step = 2*M_PI/segments;
			for (int j = 0; j < segments; j++){
		    		geometry_msgs::PoseStamped post;
		    		post.header.seq = counter;
				counter += 1;
		    		post.header.stamp = ros::Time::now();
		    		post.header.frame_id = "map";
		    		
		    		post.pose.position.x = radius/2.0*sin(j*step) + scarf_center[0];
				post.pose.position.y = radius/2.0*cos(j*step) + scarf_center[1];
				post.pose.position.z = scarf_center[2];
				path.poses.push_back(post);
			}
		}
		path_pub.publish(path);
	}
	
	
	// Determines minimum bounding box based on rotating calipers algorithm
	Poly_bound_box bounding(const vector<Point> &vertices){
	    int n_vertices = vertices.size();

	    Poly_bound_box bounded;
	    float min_area = 0;
	    
	    Point pt1;
	    Point pt2;
	    float rad;
	    Point temp_pt;
	    float area;
	    // Iterate through pairs of vertices to determine a 2d frame
	    for (int i = 0; i < n_vertices; i++){
		pt1 = vertices[i];
		pt2 = vertices[(i+1)%n_vertices];
		rad = -atan((pt2.y-pt1.y)/(pt2.x-pt1.x));
		
		float max_x, max_y, min_x, min_y;
		max_x = min_x = cos(rad)*vertices[0].x-sin(rad)*vertices[0].y;
		max_y = min_y = sin(rad)*vertices[0].x+cos(rad)*vertices[0].y;
		// Convert remaining vertices into new frame and determine the extreme points in the new x and y axis
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
	
	// Helper function for checking where the query point lies on a line made up of 2 points (p1 and p2)
	// +ve ---> to the left of line
	// -ve ---> to the right of line
	// 0   ---> on the line
	float sub_line_pt(Point pt1, Point pt2, Point q_pt) {
	    return ((q_pt.y-pt1.y)*(pt2.x-pt1.x) - (q_pt.x-pt1.x) *(pt2.y-pt1.y));
	}
	
	// Finding out if a point lies in a polygon (vertices) or not
	int inpoly(const Point &q_pt, const vector<Point> &vertices) {
	    float tol = 1e-6; // to account for finite precision
	    int wn = 0;  // the  winding number counter
	    const int n_vertices = vertices.size();

	    for (int i = 0; i < n_vertices; i++) {
		float point_in_line = sub_line_pt(vertices[i], vertices[(i+1)%n_vertices], q_pt);
		// Lies on the polygon but might not exactly be equal to zero due to finite precision
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
	
	// Create region of path plan from vertices of a convex polygon
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
		geometry_msgs::Point32 temp_pt;
		kd_node *temp_node;
		
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
		    	
		    	post.pose.position.x = temp_pt.x = move_pt.x;
			post.pose.position.y = temp_pt.y = move_pt.y;
			temp_node = tree.find_nearest(temp_pt, 0, tree.root);
			post.pose.position.z = temp_node->data.z;
			path.poses.push_back(post);
		        
		        prev_pt = move_pt;
		        move_pt.x += res/sqrt(2)*dir[0];
		        move_pt.y += res/sqrt(2)*dir[1];
        		//path_pub.publish(path);
		    }
    		    ROS_INFO_STREAM("Prev pt: " << prev_pt.x << ", " << prev_pt.y << "\n");
		    ROS_INFO_STREAM("Move pt: " << move_pt.x << ", " << move_pt.y << "\n");

    		    //path_pub.publish(path);
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
			    	
				post.pose.position.x = temp_pt.x = move_pt.x;
				post.pose.position.y = temp_pt.y = move_pt.y;
				temp_node = tree.find_nearest(temp_pt, 0, tree.root);
				post.pose.position.z = temp_node->data.z;
				path.poses.push_back(post);

		    	}
		    }
		    
		    //path_pub.publish(path);
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
	
	// Following end effector of effector on opposite side of segmentation wall
	void follow_scarf(){
		if (is_scarf_execute == true){
			robot_state::RobotStatePtr scarf_state = (*arm2_move_group_interface).getCurrentState();
			const Eigen::Isometry3d& scarf_end = scarf_state->getGlobalLinkTransform("arm2_wrist_3_link");
			
			ROS_INFO_STREAM("Trying to follow\n");
			
			target_pose.position.x = (scarf_end.translation())(0) + 0.4;
			target_pose.position.y = (scarf_end.translation())(1);
			target_pose.position.z = (scarf_end.translation())(2);
			target_pose.orientation.x = -0.2705981;
			target_pose.orientation.y = 0.6532815;
			target_pose.orientation.z = 0.6532815;
			target_pose.orientation.w = 0.2705981;
			
			std::vector<geometry_msgs::Pose> waypoints;
			waypoints.push_back(target_pose);
			moveit_msgs::RobotTrajectory trajectory;
			const double jump_threshold = 0.0;
			const double eef_step = 0.01;
			double fraction = (*arm1_move_group_interface).computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
			// Removal of last trajectory point due to non-increasing time_from_start
			trajectory.joint_trajectory.points.pop_back();
			arm1_command_pub.publish(trajectory.joint_trajectory);
		}
			
	}
};


namespace rvt = rviz_visual_tools;


int main(int argc, char** argv)
{
	// Node setup
	ros::init(argc, argv, "one_arm_move_viz");
	ros::AsyncSpinner spinner(1);
	spinner.start();
	
	std::string arm1_grp = "ur5_arm1"; // ur5_arm2 -> eff, ur5_arm1 -> viz
	moveit::planning_interface::MoveGroupInterface move0(arm1_grp);
	moveit::planning_interface::PlanningSceneInterface scene0;
	
	std::string arm2_grp = "ur5_arm2"; // ur5_arm2 -> eff, ur5_arm1 -> viz
	moveit::planning_interface::MoveGroupInterface move1(arm2_grp);
	
	eval evaluator(arm1_grp, &move0, &scene0, &move1);
	
	// Obtain proper view of workpiece at -x 0.5 -y 0.5
	// Euler xyz -x 90 -y 135 -z -45
	float scarf_center[7] = {-0.2, 0.2, 0.8, 0.0, 0.7071068, 0.5, 0.5 };
	//float scarf_center[7] = {-0.2, 0.2, 0.8, -0.2705981, 0.6532815, 0.6532815, 0.2705981};

	// Goal pose to move to
	geometry_msgs::Pose temp_pose;
	temp_pose.position.x = scarf_center[0];
	temp_pose.position.y = scarf_center[1];
	temp_pose.position.z = scarf_center[2];
	temp_pose.orientation.x = scarf_center[3];
	temp_pose.orientation.y = scarf_center[4];
	temp_pose.orientation.z = scarf_center[5];
	temp_pose.orientation.w = scarf_center[6];
	
	evaluator.set_goal(temp_pose);
	evaluator.move_to_goal();
	evaluator.kd_tree_maker = 0;
	ros::Duration(5.0).sleep();
	
	// Circle path plan
	float head_diameter = 0.01;
	float scarf_diameter = 0.2;
	int resolution = 36;
	//evaluator.create_circ_path_plan(head_diameter, scarf_diameter, scarf_center, resolution);
	//ros::Duration(3.0).sleep();
	
	// Polygon path plan
	vector<vector<float>> vertices = {{-0.63, 0.3}, {-0.54, 0.4}, {-0.53, 0.52}, {-0.68, 0.5}, {-0.68, 0.35}};
	evaluator.create_poly_path_plan(vertices);
	ros::Duration(3.0).sleep();
	
	while(ros::ok()){
		evaluator.follow_scarf(); // follow rate with a delay of about 0.1s
		//ros::Duration(0.1).sleep();
		//ros::spinOnce();
	}
	return 0;
}
