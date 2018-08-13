// Voxel Grid, PassThrough, and Statistical Outlier Removal filters applied respectively
// to the raw point cloud coming from zed camera, and filtered point cloud published
// http://pointclouds.org/documentation/tutorials/voxel_grid.php
// http://pointclouds.org/documentation/tutorials/passthrough.php
// http://pointclouds.org/documentation/tutorials/statistical_outlier.php

// Import dependencies
#include <ros/ros.h>
#include <string>
#include <iostream>
// PCL
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
// Filters
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

// Definitions
ros::Publisher pub;

std::string subscribed_topic;
std::string published_topic;

double leaf_size_x, leaf_size_y, leaf_size_z; // Voxel Filter parameters

double min_value_x, max_value_x, // PassThrough Filter parameters
       min_value_y, max_value_y,
       min_value_z, max_value_z;

int meanK; // Statistical Outlier Removal Filter parameters
double mulThresh; // Statistical Outlier Removal Filter parameters

// callback functions

void altitude_cb(const double msg)
{
    min_value_z = msg;
}
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    // Create point cloud and message objects
    pcl::PCLPointCloud2* raw_cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr1(raw_cloud);
    pcl::PCLPointCloud2 first_cloud;
    pcl::PCLPointCloud2 second_cloud;
    pcl::PCLPointCloud2 third_cloud;
    sensor_msgs::PointCloud2 carrier;

    pcl_conversions::toPCL(*cloud_msg, *raw_cloud); // Convert to PCL data type

    // Perform the VoxelGrid Filter
    pcl::VoxelGrid<pcl::PCLPointCloud2> v_filter;
    v_filter.setInputCloud(cloudPtr1); // Pass raw_cloud to the filter
    v_filter.setLeafSize(leaf_size_x, leaf_size_y, leaf_size_z); // Set leaf size
    v_filter.filter(first_cloud); // Store output data in first_cloud

    pcl_conversions::moveFromPCL(first_cloud, carrier); // Convert to ROS data type
    pcl::PCLPointCloud2* voxel_cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr2(voxel_cloud);
    pcl_conversions::toPCL(carrier, *voxel_cloud); // Convert to PCL data type

    // Perform the PassThrough Filter to the X axis
    pcl::PassThrough<pcl::PCLPointCloud2> px_filter;
    px_filter.setInputCloud(cloudPtr2); // Pass filtered_cloud to the filter
    px_filter.setFilterFieldName("x"); // Set axis x
    px_filter.setFilterLimits(min_value_x, max_value_x); // Set limits min_value to max_value
    px_filter.filter(second_cloud); // Restore output data in second_cloud

    pcl_conversions::moveFromPCL(second_cloud, carrier); // Convert to ROS data type
    pcl::PCLPointCloud2* x_removed_cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr_X(x_removed_cloud);
    pcl_conversions::toPCL(carrier, *x_removed_cloud); // Convert to PCL data type

    // Perform the PassThrough Filter to the Y axis
    pcl::PassThrough<pcl::PCLPointCloud2> py_filter;
    py_filter.setInputCloud(cloudPtr_X); // Pass filtered_cloud to the filter
    py_filter.setFilterFieldName("y"); // Set axis y
    py_filter.setFilterLimits(min_value_y, max_value_y); // Set limits min_value to max_value
    py_filter.filter(second_cloud); // Restore output data in second_cloud

    pcl_conversions::moveFromPCL(second_cloud, carrier); // Convert to ROS data type
    pcl::PCLPointCloud2* y_removed_cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr_Y(y_removed_cloud);
    pcl_conversions::toPCL(carrier, *y_removed_cloud); // Convert to PCL data type


    // Perform the PassThrough Filter to the Z axis
    pcl::PassThrough<pcl::PCLPointCloud2> pz_filter;
    pz_filter.setInputCloud(cloudPtr_Y); // Pass filtered_cloud to the filter
    pz_filter.setFilterFieldName("z"); // Set axis z
    pz_filter.setFilterLimits(min_value_z, max_value_z); // Set limits min_value to max_value
    pz_filter.filter(second_cloud); // Restore output data in second_cloud

    pcl_conversions::moveFromPCL(second_cloud, carrier); // Convert to ROS data type
    pcl::PCLPointCloud2* pass_cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr3(pass_cloud);
    pcl_conversions::toPCL(carrier, *pass_cloud); // Convert to PCL data type

    // Perform the Statistical Outlier Removal Filter
    pcl::StatisticalOutlierRemoval<pcl::PCLPointCloud2> s_filter;
    s_filter.setInputCloud(cloudPtr3); // Pass filtered_cloud to the filter
    s_filter.setMeanK(meanK); // Set the number of neighbors to analyze for each point
    s_filter.setStddevMulThresh(mulThresh); // Set standard deviation multiplier
    s_filter.filter(third_cloud); // Restore output data in filtered_cloud

    pcl_conversions::moveFromPCL(third_cloud, carrier); // Convert to ROS data type

    pub.publish(carrier); // Publish filtered_cloud to the topic
}

// main function
int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv, "filter");
    ros::NodeHandle n;

    // Load parameters from launch file
    ros::NodeHandle nh_private("~");
    // Topics
    nh_private.param<std::string>("subscribed_topic", subscribed_topic, "/point_cloud/cloud_registered");
    nh_private.param<std::string>("published_topic", published_topic, "cloud_filtered");
    // Voxel Filter Parameters
    nh_private.param<double>("leaf_size_x", leaf_size_x, 0.05);
    nh_private.param<double>("leaf_size_y", leaf_size_y, 0.05);
    nh_private.param<double>("leaf_size_z", leaf_size_z, 0.05);
    // PassThrough Filter Parameters
    nh_private.param<double>("min_value_x", min_value_x, 0.5);
    nh_private.param<double>("max_value_x", max_value_x, 18.0);
    nh_private.param<double>("min_value_y", min_value_y, -10.0);
    nh_private.param<double>("max_value_y", max_value_y, 10.0);
    // nh_private.param<double>("min_value_z", min_value_z, -10.0);
    nh_private.param<double>("max_value_z", max_value_z, 10.0);
    // Statistical Outlier Removal Filter Parameters
    nh_private.param<int>("meanK", meanK, 64);
    nh_private.param<double>("mulThresh", mulThresh, 1.5);

    // Create Subscriber and listen subscribed_topic
    ros::Subscriber sub = n.subscribe<sensor_msgs::PointCloud2>(subscribed_topic, 1, cloud_cb);
    ros::Subscriber laser = n.subscribe<double>(altitude_tracker, 1, altitude_cb);

    // Create Publisher
    pub = n.advertise<sensor_msgs::PointCloud2>(published_topic, 1);

    // Spin
    ros::spin();
    return 0;
}
