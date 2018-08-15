// Iterative Closest Point Algorithm

// Import dependencies
#include <ros/ros.h>
#include <string>
#include <iostream>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/registration/icp.h>

// Definitions
ros::Publisher pub;
std::string subscribed_topic;
std::string published_topic;
int meanK;
double mulThresh;

// callback
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    pcl::PCLPointCloud2* raw_cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr(raw_cloud);

    pcl::PointCloud<pcl::PointXYZ> *smooth_cloud = new pcl::PointCloud<pcl::PointXYZ>;
    pcl::PointCloud<pcl::PointXYZ>::Ptr smoothCloudPtr(smooth_cloud);

    // Convert to PCL data type
    pcl_conversions::toPCL(*cloud_msg, *raw_cloud);
    pcl::fromPCLPointCloud2(*cloudPtr, *smoothCloudPtr);

    // Convert to PCL data type
    pcl_conversions::toPCL(*cloud_msg, *passed_cloud);

    IterativeClosestPoint<PointXYZ, PointXYZ> icp;
    // Set the input source and target
    icp.setInputCloud (cloud_source);
    icp.setInputTarget (cloud_target);
    // Set the max correspondence distance to 5cm
    icp.setMaxCorrespondenceDistance (0.05);
    // Set the maximum number of iterations (criterion 1)
    icp.setMaximumIterations (50);
    // Set the transformation epsilon (criterion 2)
    icp.setTransformationEpsilon (1e-8);
    // Set the euclidean distance difference epsilon (criterion 3)
    icp.setEuclideanFitnessEpsilon (1);
    // Perform the alignment
    icp.align (cloud_source_registered);
    // Obtain the transformation that aligned cloud_source to cloud_source_registered
    Eigen::Matrix4f transformation = icp.getFinalTransformation ();

    // Convert to ROS data type
    sensor_msgs::PointCloud2 output_cloud;
    pcl_conversions::moveFromPCL(cleaned_cloud, output_cloud);

    pub.publish(output_cloud); // Publish cleaned_cloud to the /cloud_cleaned topic
}

// main
int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv, "statistical_removal");
    ros::NodeHandle n;

    // Load parameters from launch file
    ros::NodeHandle nh_private("~");
    nh_private.param<std::string>("subscribed_topic", subscribed_topic, "/cloud_passed");
    nh_private.param<std::string>("published_topic", published_topic, "cloud_cleaned");

    // Create Subscriber and listen /cloud_passed topic
    ros::Subscriber sub = n.subscribe<sensor_msgs::PointCloud2>(subscribed_topic, 1, cloud_cb);

    // Create Publisher
    pub = n.advertise<sensor_msgs::PointCloud2>(published_topic, 1);

    // Spin
    ros::spin();
    return 0;
}
