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
std::string subscribed_topic;
std::string published_topic;
std::string field_name;
double min_value, max_value;

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
    p_filter.setFilterFieldName(field_name); // Set axis
    p_filter.setFilterLimits(min_value, max_value); // Set limits min_value to max_value
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

    // Load parameters from launch file
    ros::NodeHandle nh_private("~");
    nh_private.param<std::string>("subscribed_topic", subscribed_topic, "/cloud_downsampled");
    nh_private.param<std::string>("published_topic", published_topic, "cloud_passed");
    nh_private.param<std::string>("field_name", field_name, "x");
    nh_private.param<double>("min_value", min_value, 0.5);
    nh_private.param<double>("max_value", max_value, 18.0);

    // Create Subscriber and listen /cloud_downsampled topic
    ros::Subscriber sub = n.subscribe<sensor_msgs::PointCloud2>(subscribed_topic, 1, cloud_cb);

    // Create Publisher
    pub = n.advertise<sensor_msgs::PointCloud2>(published_topic, 1);

    // Spin
    ros::spin();
    return 0;
}
