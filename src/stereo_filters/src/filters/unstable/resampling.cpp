// Smoothing and normal estimation based on polynomial reconstruction
// Resampling using Moving Least Squares (MLS) surface reconstruction method
// http://pointclouds.org/documentation/tutorials/resampling.php

// Import dependencies
#include <string>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>

// Definitions
ros::Publisher pub;
std::string subscribed_topic;
std::string published_topic;

// callback
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    // Create point cloud objects
    pcl::PCLPointCloud2* raw_cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr(raw_cloud);

    pcl::PointCloud<pcl::PointXYZ> *smooth_cloud = new pcl::PointCloud<pcl::PointXYZ>;
    pcl::PointCloud<pcl::PointXYZ>::Ptr smoothCloudPtr(smooth_cloud);

    // Convert to PCL data type
    pcl_conversions::toPCL(*cloud_msg, *raw_cloud);
    pcl::fromPCLPointCloud2(*cloudPtr, *smoothCloudPtr);

    // Create a KD-Tree
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

    // Output has the PointNormal type in order to store the normals calculated by MLS
    pcl::PointCloud<pcl::PointNormal> mls_points;

    // Init object (second point type is for the normals, even if unused)
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
    mls.setComputeNormals(true);
    mls.setInputCloud(smoothCloudPtr);
    mls.setPolynomialFit(true);
    mls.setSearchMethod(tree);
    mls.setSearchRadius(0.03);

    // Reconstruct
    mls.process(mls_points);

    // Publish
    pub.publish(mls_points);
}

// main
int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv, "resampling");
    ros::NodeHandle n;

    // Load parameters from launch file
    ros::NodeHandle nh_private("~");
    nh_private.param<std::string>("subscribed_topic", subscribed_topic, "/cloud_filtered");
    nh_private.param<std::string>("published_topic", published_topic, "cloud_smooth");

    // Create Subscriber and listen the subscribed_topic
    ros::Subscriber sub = n.subscribe<sensor_msgs::PointCloud2>(subscribed_topic, 1, cloud_cb);

    // Create Publisher
    pub = n.advertise<sensor_msgs::PointCloud2>(published_topic, 1);

    // Spin
    ros::spin();
    return 0;
}
