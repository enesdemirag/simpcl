// Voxel Grid, PassThrough, and Statistical Outlier Removal filters applied respectively
// to the raw point cloud coming from zed camera, and filtered point cloud published

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

std::string field_name; // PassThrough Filter parameters
double min_value, max_value; // PassThrough Filter parameters

int meanK; // Statistical Outlier Removal Filter parameters
double mulThresh; // Statistical Outlier Removal Filter parameters

// callback function
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    // Create point cloud objects
    pcl::PCLPointCloud2* raw_cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr(raw_cloud);
    pcl::PCLPointCloud2 filtered_cloud;

    // Convert to PCL data type
    pcl_conversions::toPCL(*cloud_msg, *raw_cloud);

    // Perform the VoxelGrid Filter
    pcl::VoxelGrid<pcl::PCLPointCloud2> v_filter;
    v_filter.setInputCloud(cloudPtr); // Pass raw_cloud to the filter
    v_filter.setLeafSize(leaf_size_x, leaf_size_y, leaf_size_z); // Set leaf size
    v_filter.filter(filtered_cloud); // Store output data in filtered_cloud

    // Perform the PassThrough Filter
    pcl::PassThrough<pcl::PCLPointCloud2> p_filter;
    p_filter.setInputCloud(filtered_cloud); // Pass filtered_cloud to the filter
    p_filter.setFilterFieldName(field_name); // Set axis
    p_filter.setFilterLimits(min_value, max_value); // Set limits min_value to max_value
    p_filter.filter(filtered_cloud); // Restore output data in filtered_cloud

    // Perform the Statistical Outlier Removal Filter
    pcl::StatisticalOutlierRemoval<pcl::PCLPointCloud2> s_filter;
    s_filter.setInputCloud(filtered_cloud); // Pass filtered_cloud to the filter
    s_filter.setMeanK(meanK); // Set the number of neighbors to analyze for each point
    s_filter.setStddevMulThresh(mulThresh); // Set standard deviation multiplier
    s_filter.filter(filtered_cloud); // Restore output data in filtered_cloud

    // Convert to ROS data type
    sensor_msgs::PointCloud2 output_cloud;
    pcl_conversions::moveFromPCL(filtered_cloud, output_cloud);

    pub.publish(output_cloud); // Publish filtered_cloud to the topic
}

// main function
int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv, "multi_filter");
    ros::NodeHandle n;

    // Load parameters from launch file
    ros::NodeHandle nh_private("~");
    // Topics
    nh_private.param<std::string>("subscribed_topic", subscribed_topic, "/point_cloud/cloud_registered");
    nh_private.param<std::string>("published_topic", published_topic, "cloud_filtered");
    // Voxel Filter Parameters
    nh_private.param<double>("leaf_size_x", leaf_size_x, 0.05);
    nh_private.param<double>("leaf_size_y", leaf_size_y, 0.05);
    nh_private.param<double>("leaf_size_z", leaf_size_z, 0.15);
    // PassThrough Filter Parameters
    nh_private.param<std::string>("field_name", field_name, "x");
    nh_private.param<double>("min_value", min_value, 0.5);
    nh_private.param<double>("max_value", max_value, 18.0);
    // Statistical Outlier Removal Filter Parameters
    nh_private.param<int>("meanK", meanK, 75);
    nh_private.param<double>("mulThresh", mulThresh, 1.25);

    // Create Subscriber and listen subscribed_topic
    ros::Subscriber sub = n.subscribe<sensor_msgs::PointCloud2>(subscribed_topic, 1, cloud_cb);

    // Create Publisher
    pub = n.advertise<sensor_msgs::PointCloud2>(published_topic, 1);

    // Spin
    ros::spin();
    return 0;
}
