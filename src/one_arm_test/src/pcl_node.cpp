// Include the ROS library
#include <ros/ros.h>

// Include pcl
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>

// Include PointCloud2 message
#include <sensor_msgs/PointCloud2.h>

// Include tf
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/convert.h>
#include <geometry_msgs/TransformStamped.h>

// Topics
static const std::string IMAGE_TOPIC = "arm1/camera1/depth/points";
static const std::string PUBLISH_TOPIC = "/pcl/points";

// ROS Publisher
ros::Publisher pub;

// TF Buffer
tf2_ros::Buffer tfBuffer;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr&);

int main (int argc, char** argv){
	// Initialize the ROS Node "roscpp_pcl_example"
	ros::init (argc, argv, "pcl_node");
	ros::NodeHandle nh;
	
	// tf listener
	tf2_ros::TransformListener tfListener(tfBuffer);

	// Create a ROS Subscriber to IMAGE_TOPIC with a queue_size of 1 and a callback function to cloud_cb
	ros::Subscriber sub = nh.subscribe(IMAGE_TOPIC, 1, cloud_cb);

	// Create a ROS publisher to PUBLISH_TOPIC with a queue_size of 1
	pub = nh.advertise<sensor_msgs::PointCloud2>(PUBLISH_TOPIC, 1);

	// Spin
	ros::spin();

	// Success
	return 0;
}

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg){
	// Container for original & filtered data
	pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
	pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
	pcl::PCLPointCloud2 cloud_filtered;
	pcl::PCLPointCloud2 cloud_framed;

	// Convert to PCL data type
	pcl_conversions::toPCL(*cloud_msg, *cloud);

	// Perform the actual filtering
	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	sor.setInputCloud (cloudPtr);
	sor.setLeafSize (0.01, 0.01, 0.01);
	sor.filter (cloud_filtered);
	
	// Convert to ROS data type
	sensor_msgs::PointCloud2 output;
	pcl_conversions::moveFromPCL(cloud_filtered, output);
	
	
	// Perform frame transformation
        geometry_msgs::TransformStamped transformStamped;
	try{
		transformStamped = tfBuffer.lookupTransform("map", "arm1/camera_depth_optical_frame", ros::Time(0));
		
		//Eigen::Translation3f tl_btol(transform.getOrigin().getX(), transform.getOrigin().getY(), transform.getOrigin().getZ());
		//double roll, pitch, yaw;
		//tf::Matrix3x3(transform.getRotation()).getEulerYPR(yaw, pitch, roll);
		//Eigen::AngleAxisf rot_x_btol(roll, Eigen::Vector3f::UnitX());
		//Eigen::AngleAxisf rot_y_btol(pitch, Eigen::Vector3f::UnitY());
		//Eigen::AngleAxisf rot_z_btol(yaw, Eigen::Vector3f::UnitZ());
		//Eigen::Matrix4f transform_matrix = (tl_btol * rot_z_btol *rot_y_btol * rot_x_btol).matrix();
		
		sensor_msgs::PointCloud2 output_transformed;
		tf2::doTransform(output, output_transformed, transformStamped);
		
		// Publish the data
		pub.publish(output_transformed);
	}catch(...){
		// Publish the data
		pub.publish(output);
	}
	
	//pub.publish(output);
}
