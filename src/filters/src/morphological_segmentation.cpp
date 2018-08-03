// Identifying ground returns using "ProgressiveMorphologicalFilter" segmentation
// http://pointclouds.org/documentation/tutorials/progressive_morphological_filtering.php

// Import Dependencies
#include <string>
#include <ros/ros.h>
#include <iostream>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/progressive_morphological_filter.h>

// Definitions
ros::Publisher pub;
std::string subscribed_topic;
std::string published_topic;
int maxWindowSize;
double slope, initialDistance, maxDistance;

// callback
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    // Create point cloud objects for raw cloud, filtered cloud, and ground_cloud
    pcl::PCLPointCloud2* raw_cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr(raw_cloud);
    pcl::PCLPointCloud2 filtered_cloud;
    pcl::PointIndicesPtr ground_cloud(new pcl::PointIndices);

    // Convert to PCL data type
    pcl_conversions::toPCL(*cloud_msg, *raw_cloud);

    // Perform the Progressive Morphological Filter
    pcl::ProgressiveMorphologicalFilter<pcl::PointXYZ> pmf;
    pmf.setInputCloud(cloudPtr); // Pass raw_cloud to the filter
    pmf.setMaxWindowSize(maxWindowSize); // Set maximum window size
    pmf.setSlope(slope); // Set slope
    pmf.setInitialDistance(initialDistance); // Set initial distance
    pmf.setMaxDistance(maxDistance); // Set maximum distance
    pmf.extract(ground_cloud -> indices);

    // Perform extraction
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloudPtr); // Pass raw_cloud to the filter
    extract.setIndices(ground_cloud); // Extract indices from raw_cloud and store in ground_cloud
    // extract.setNegative(true); If you want to get just ground
    extract.filter(filtered_cloud); // Store output data in filtered_cloud

    // Convert to ROS data type
    sensor_msgs::PointCloud2 output_cloud;
    pcl_conversions::moveFromPCL(filtered_cloud, output_cloud);

    pub.publish(output_cloud); // Publish filtered_cloud to the /cloud_without_ground topic
}
// main
int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv, "morphological_segmentation");
    ros::NodeHandle n;

    // Load parameters from launch file
    ros::NodeHandle nh_private("~");
    nh_private.param<std::string>("subscribed_topic", subscribed_topic, "/cloud_filtered");
    nh_private.param<std::string>("published_topic", published_topic, "cloud_without_ground");
    nh_private.param<int>("maxWindowSize", maxWindowSize, 20);
    nh_private.param<double>("slope", slope, 1.0);
    nh_private.param<double>("initialDistance", initialDistance, 0.5);
    nh_private.param<double>("maxDistance", maxDistance, 3.0);

    // Create Subscriber and listen subscribed_topic
    ros::Subscriber sub = n.subscribe<sensor_msgs::PointCloud2>(subscribed_topic, 1, cloud_cb);

    // Create Publisher
    pub = n.advertise<sensor_msgs::PointCloud2>(published_topic, 1);

    // Spin
    ros::spin();
    return 0;
}
