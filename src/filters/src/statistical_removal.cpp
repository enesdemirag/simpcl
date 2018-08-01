// Remove noise with "Statistical Outlier Removal Filter"
// http://pointclouds.org/documentation/tutorials/statistical_outlier.php

// Import dependencies
#include <ros/ros.h>
#include <string>
#include <iostream>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/statistical_outlier_removal.h>

// Definitions
ros::Publisher pub;
std::string subscribed_topic;
std::string published_topic;
int meanK;
double mulThresh;

// callback
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    //Create point cloud objects for passed_cloud and cleaned_cloud
    pcl::PCLPointCloud2* passed_cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr(passed_cloud);
    pcl::PCLPointCloud2 cleaned_cloud;

    // Convert to PCL data type
    pcl_conversions::toPCL(*cloud_msg, *passed_cloud);

    // Perform the Statistical Outlier Removal Filter
    pcl::StatisticalOutlierRemoval<pcl::PCLPointCloud2> s_filter;
    s_filter.setInputCloud(cloudPtr); // Pass passed_cloud to the filter
    s_filter.setMeanK(meanK); // The number of neighbors to analyze for each point is set to 50
    s_filter.setStddevMulThresh(mulThresh); // Standard deviation multiplier set to 1.0
    s_filter.filter(cleaned_cloud); // Store output data in cleaned_cloud

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
    nh_private.param<int>("meanK", meanK, 75);
    nh_private.param<double>("mulThresh", mulThresh, 1.25);

    // Create Subscriber and listen /cloud_passed topic
    ros::Subscriber sub = n.subscribe<sensor_msgs::PointCloud2>(subscribed_topic, 1, cloud_cb);

    // Create Publisher
    pub = n.advertise<sensor_msgs::PointCloud2>(published_topic, 1);

    // Spin
    ros::spin();
    return 0;
}
