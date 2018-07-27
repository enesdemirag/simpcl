// Downsampling with "Voxel Grid Filter"
// http://pointclouds.org/documentation/tutorials/voxel_grid.php

// Import dependencies
#include <ros/ros.h>
#include <iostream>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/voxel_grid.h>

// Definitions
//typedef pcl::PCLPointCloud2 PC2;
ros::Publisher pub;

// callback
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    //Create point cloud object for raw and downsampled_cloud
    pcl::PCLPointCloud2* raw_cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr(raw_cloud);
    pcl::PCLPointCloud2 downsampled_cloud;

    // Convert to PCL data type
    pcl_conversions::toPCL(*cloud_msg, *raw_cloud);

    // Perform the VoxelGrid Filter
    pcl::VoxelGrid<pcl::PCLPointCloud2> v_filter;
    v_filter.setInputCloud(raw_cloud); // Pass raw_cloud to the filter
    v_filter.setLeafSize(0.01f, 0.01f, 0.01f); // Set leaf size of 1cm
    v_filter.filter(downsampled_cloud); // Store output data in downsampled_cloud

    // Convert to ROS data type
    sensor_msgs::PointCloud2 output;
    pcl_conversions::moveFromPCL(downsampled_cloud, output_cloud);

    pub.publish(output_cloud); // Publish downsampled_cloud to the /cloud_downsampled topic
}

// main
int main(int argc, char** argv[])
{
    // Initialize ROS
    ros::init(argc, argv, "volex_filter");
    ros::NodeHandle n;
    //ros::Rate loop_rate(60);

    // Create Subscriber and listen /zed/point_cloud/cloud_registered topic
    ros::Subscriber sub = n.subscribe<sensor_msgs::PointCloud2>("/zed/point_cloud/cloud_registered", 1, cloud_cb);

    // Create Publisher
    pub = n.advertise<sensor_msgs::PointCloud2>("cloud_downsampled", 1);

    // Spin
    ros::spin();
    //ros::spinOnce();
    //loop_rate.sleep();
    return 0;
}
