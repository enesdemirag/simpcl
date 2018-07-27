// Downsampling with "Voxel Grid Filter"
// http://pointclouds.org/documentation/tutorials/voxel_grid.php

// Import dependencies
#include <ros/ros.h>
#include <iostream>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <sensor_msgs/PointCloud2.h>

// Definitions
typedef pcl::PCLPointCloud2 PC2;

// callback
void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& raw_cloud)
{
    //Create point cloud object for downsampled_cloud
    PC2::Ptr downsampled_cloud (new PC2);
    downsampled_cloud->header.frame_id = "volex_tf_frame";
    downsampled_cloud->height = downsampled_cloud->width = 1;

    // Create the VoxelGrid Filter
    pcl::VoxelGrid<pcl::PCLPointCloud2> v_filter;
    v_filter.setInputCloud (raw_cloud); // Pass raw_cloud to the filter
    v_filter.setLeafSize (0.01f, 0.01f, 0.01f); // Set leaf size of 1cm
    v_filter.filter (*downsampled_cloud); // Store output data in downsampled_cloud
}

// main
int main(int argc, char** argv[])
{
    // Create node
    ros::init(argc, argv, "volex_filter");
    ros::NodeHandle n;
    ros::Rate loop_rate(60);

    // Create Publisher
    ros::Publisher pub = n.advertise<PC2>("/cloud_downsampled", 1);

    // Create Subscriber and listen /zed/point_cloud/cloud_registered topic
    ros::Subscriber sub = n.subscribe<PC2>("/zed/point_cloud/cloud_registered", 1, cloud_cb);

    pub.publish(downsampled_cloud); // Publish downsampled_cloud to the /cloud_downsampled topic

    // Kill node
    ros::spinOnce();
    loop_rate.sleep();
    return 0;
}
