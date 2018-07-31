// Getting points between 0.5m to 18m with "Passthrough Filter"
// http://pointclouds.org/documentation/tutorials/passthrough.php

// Import dependencies
#include <ros/ros.h>
#include <string>
#include <iostream>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/passthrough.h>

// Definitions
ros::Publisher pub;
std::string subscribed_topic = "/cloud_downsampled";
std::string published_topic = "cloud_passed";

// callback
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    //Create point cloud objects for downsampled_cloud and passed_cloud
    pcl::PCLPointCloud2* downsampled_cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr(downsampled_cloud);
    pcl::PCLPointCloud2 passed_cloud;

    // Convert to PCL data type
    pcl_conversions::toPCL(*cloud_msg, *downsampled_cloud);

    // Perform the PassThrough Filter
    pcl::PassThrough<pcl::PCLPointCloud2> p_filter;
    p_filter.setInputCloud(cloudPtr); // Pass downsampled_cloud to the filter
    p_filter.setFilterFieldName("x"); // Set axis as x
    p_filter.setFilterLimits(0.5, 18.0); // Set limits 0.5m to 18m
    p_filter.filter(passed_cloud); // Store output data in passed_cloud

    // Convert to ROS data type
    sensor_msgs::PointCloud2 output_cloud;
    pcl_conversions::moveFromPCL(passed_cloud, output_cloud);

    pub.publish(output_cloud); // Publish passed_cloud to the /cloud_passed topic
}

// main
int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv, "passthrough_filter");
    ros::NodeHandle n;

    // Create Subscriber and listen /cloud_downsampled topic
    ros::Subscriber sub = n.subscribe<sensor_msgs::PointCloud2>(subscribed_topic, 1, cloud_cb);

    // Create Publisher
    pub = n.advertise<sensor_msgs::PointCloud2>(published_topic, 1);

    // Spin
    ros::spin();
    return 0;
}
