// Remove noise with "Radius Outlier Removal Filter"
// http://www.pointclouds.org/documentation/tutorials/remove_outliers.php

// Import dependencies
#include <ros/ros.h>
#include <string>
#include <iostream>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/radius_outlier_removal.h>

// Definitions
ros::Publisher pub;
std::string subscribed_topic = "/cloud_passed";
std::string published_topic = "cloud_cleaned";

// callback
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    //Create point cloud objects for passed_cloud and cleaned_cloud
    pcl::PCLPointCloud2* passed_cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr(passed_cloud);
    pcl::PCLPointCloud2 cleaned_cloud;

    // Convert to PCL data type
    pcl_conversions::toPCL(*cloud_msg, *passed_cloud);

    // Perform the Radius Outlier Removal Filter
    pcl::RadiusOutlierRemoval<pcl::PCLPointCloud2> r_filter;
    r_filter.setInputCloud(cloudPtr); // Pass passed_cloud to the filter
    r_filter.setRadiusSearch(0.5); // Set radius to 0.5m
    r_filter.setMinNeighborsInRadius(10); // Set minimum neighbor number to 10
    r_filter.filter(cleaned_cloud); // Store output data in cleaned_cloud

    // Convert to ROS data type
    sensor_msgs::PointCloud2 output_cloud;
    pcl_conversions::moveFromPCL(cleaned_cloud, output_cloud);

    pub.publish(output_cloud); // Publish cleaned_cloud to the /cloud_cleaned topic
}

// main
int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv, "radius_removal");
    ros::NodeHandle n;

    // Create Subscriber and listen /cloud_passed topic
    ros::Subscriber sub = n.subscribe<sensor_msgs::PointCloud2>(subscribed_topic, 1, cloud_cb);

    // Create Publisher
    pub = n.advertise<sensor_msgs::PointCloud2>(published_topic, 1);

    // Spin
    ros::spin();
    return 0;
}
