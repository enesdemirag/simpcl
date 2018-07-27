// Remove noise with "Statistical Outlier Removal Filter"
// http://pointclouds.org/documentation/tutorials/statistical_outlier.php

// Import dependencies
#include <ros/ros.h>
#include <iostream>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/statistical_outlier_removal.h>

// Definitions
//typedef pcl::PCLPointCloud2 PC2;
ros::Publisher pub;

// callback
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& passed_cloud)
{
    //Create point cloud object for cleaned_cloud
    sensor_msgs::PointCloud2 cleaned_cloud;

    // Create the Statistical Outlier Removal Filter
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> s_filter;
    s_filter.setInputCloud(passed_cloud); // Pass passed_cloud to the filter
    s_filter.setMeanK(50); // The number of neighbors to analyze for each point is set to 50
    s_filter.setStddevMulThresh(1.0); // Standard deviation multiplier set to 1
    s_filter.filter(*cleaned_cloud); // Store output data in cleaned_cloud

    pub.publish(cleaned_cloud); // Publish cleaned_cloud to the /cloud_cleaned topic
}

// main
int main(int argc, char** argv[])
{
    // Initialize ROS
    ros::init(argc, argv, "statistical_removal");
    ros::NodeHandle n;
    //ros::Rate loop_rate(60);

    // Create Subscriber and listen /cloud_passed topic
    ros::Subscriber sub = n.subscribe("/cloud_passed", 1, cloud_cb);

    // Create Publisher
    pub = n.advertise<sensor_msgs::PointCloud2>("/cloud_cleaned", 1);

    // Spin
    ros::spin();
    //ros::spinOnce();
    //loop_rate.sleep();
    return 0;
}
