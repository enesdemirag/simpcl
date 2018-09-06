// Identifying ground returns using "ProgressiveMorphologicalFilter" segmentation
// http://pointclouds.org/documentation/tutorials/progressive_morphological_filtering.php

// Import Dependencies
#include <string>
#include <ros/ros.h>
#include <iostream>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/conversions.h>
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
    pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointIndicesPtr ground (new pcl::PointIndices);

    // Perform the Progressive Morphological Filter
    pcl::ProgressiveMorphologicalFilter<pcl::PointXYZ> pmf;
    pmf.setInputCloud(raw_cloud); // Pass raw_cloud to the filter
    pmf.setMaxWindowSize(maxWindowSize); // Set maximum window size
    pmf.setSlope(slope); // Set slope
    pmf.setInitialDistance(initialDistance); // Set initial distance
    pmf.setMaxDistance(maxDistance); // Set maximum distance
    pmf.extract(ground -> indices);

    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(raw_cloud);
    extract.setIndices(ground);
    extract.setNegative(true);
    extract.filter(*cloud_filtered);

    // Convert to ROS data type
    pcl::PCLPointCloud2 x_cloud; //  = new pcl::PCLPointCloud2;

    sensor_msgs::PointCloud2 output_cloud;

    pcl::toPCLPointCloud2(*cloud_filtered, x_cloud);

    pcl_conversions::moveFromPCL(x_cloud, output_cloud);
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
    nh_private.param<std::string>("subscribed_topic", subscribed_topic, "/point_cloud");
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
