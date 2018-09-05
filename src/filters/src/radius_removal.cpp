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
std::string subscribed_topic;
std::string published_topic;
double radius;
int minNeighbors;

pcl::PCLPointCloud2* passed_cloud = new pcl::PCLPointCloud2;
pcl::PCLPointCloud2ConstPtr cloudPtr(passed_cloud);
pcl::PCLPointCloud2 cleaned_cloud;

sensor_msgs::PointCloud2 output_cloud;

// callback
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    // Convert to PCL data type
    pcl_conversions::toPCL(*cloud_msg, *passed_cloud);
}

// main
int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv, "radius_removal");
    ros::NodeHandle n;
    ros::Rate r(10);

    // Load parameters from launch file
    ros::NodeHandle nh_private("~");
    nh_private.param<std::string>("subscribed_topic", subscribed_topic, "/cloud_passed");
    nh_private.param<std::string>("published_topic", published_topic, "cloud_cleaned");
    nh_private.param<double>("radius", radius, 0.5);
    nh_private.param<int>("minNeighbors", minNeighbors, 100);

    // Create Subscriber and listen /cloud_passed topic
    ros::Subscriber sub = n.subscribe<sensor_msgs::PointCloud2>(subscribed_topic, 1, cloud_cb);

    // Create Publisher
    pub = n.advertise<sensor_msgs::PointCloud2>(published_topic, 1);

    while(ros::ok())
    {
        // Perform the Radius Outlier Removal Filter
        pcl::RadiusOutlierRemoval<pcl::PCLPointCloud2> r_filter;
        r_filter.setInputCloud(cloudPtr); // Pass passed_cloud to the filter
        r_filter.setRadiusSearch(radius); // Set radius
        r_filter.setMinNeighborsInRadius(minNeighbors); // Set minimum neighbor number
        r_filter.filter(cleaned_cloud); // Store output data in cleaned_cloud
        pcl_conversions::moveFromPCL(cleaned_cloud, output_cloud);
        pub.publish(output_cloud); // Publish cleaned_cloud to the /cloud_cleaned topic

        r.sleep();
        ros::spinOnce();
    }
    return 0;
}
