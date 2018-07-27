// Getting points between 0.5m to 15m with "Passthrough Filter"
// http://pointclouds.org/documentation/tutorials/passthrough.php

// Import dependencies
#include <ros/ros.h>
#include <iostream>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/passthrough.h>

// Definitions
//typedef pcl::PCLPointCloud2 PC2;
ros::Publisher pub;

// callback
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& downsampled_cloud)
{
    //Create point cloud object for passed_cloud
    sensor_msgs::PointCloud2 passed_cloud;

    // Create the PassThrough Filter
    pcl::PassThrough<pcl::PointXYZ> p_filter;
    p_filter.setInputCloud(downsampled_cloud); // Pass downsampled_cloud to the filter
    p_filter.setFilterFieldName("z"); // Set axis
    p_filter.setFilterLimits(0.5, 15.0); // Set limits 0.5m to 15m
    p_filter.filter(*passed_cloud); // Store output data in passed_cloud

    pub.publish(passed_cloud); // Publish passed_cloud to the /cloud_passed topic
}

// main
int main(int argc, char** argv[])
{
    // Initialize ROS
    ros::init(argc, argv, "passthrough_filter");
    ros::NodeHandle n;
    //ros::Rate loop_rate(60);

    // Create Subscriber and listen /cloud_downsampled topic
    ros::Subscriber sub = n.subscribe("/cloud_downsampled", 1, cloud_cb);

    // Create Publisher
    pub = n.advertise<sensor_msgs::PointCloud2>("/cloud_passed", 1);

    // Spin
    ros::spin();
    //ros::spinOnce();
    //loop_rate.sleep();
    return 0;
}
