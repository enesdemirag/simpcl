// Projecting points using a parametric model
// Plane Model used.
// http://www.pointclouds.org/documentation/tutorials/project_inliers.php

// Import dependencies
#include <string>
#include <ros/ros.h>
#include <iostream>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>

// Definitions
ros::Publisher pub;
std::string subscribed_topic;
std::string published_topic;

// callback
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    //Create point cloud objects for cleaned_cloud and projected_cloud
    pcl::PCLPointCloud2* cleaned_cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr(cleaned_cloud);
    pcl::PCLPointCloud2 projected_cloud;

    // Convert to PCL data type
    pcl_conversions::toPCL(*cloud_msg, *cleaned_cloud);

    // Fill the ModelCoefficients values
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    coefficients->values.resize(4);
    coefficients->values[0] = 0;
    coefficients->values[1] = 0;
    coefficients->values[2] = 1.0;
    coefficients->values[3] = 0;

    // Perform Projection on a plane model
    pcl::ProjectInliers<pcl::PCLPointCloud2> proj;
    proj.setModelType (pcl::SACMODEL_PLANE); // Set model to plane
    proj.setInputCloud(cloudPtr); // cleaned_cloud to the filter
    proj.setModelCoefficients(coefficients);
    proj.filter(projected_cloud); // Store output data in projected_cloud

    // Convert to ROS data type
    sensor_msgs::PointCloud2 output_cloud;
    pcl_conversions::moveFromPCL(projected_cloud, output_cloud);

    pub.publish(output_cloud); // Publish projected_cloud to the /cloud_projected topic
}

int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv, "model_projection");
    ros::NodeHandle n;

    // Load parameters from launch file
    ros::NodeHandle nh_private("~");
    nh_private.param<std::string>("subscribed_topic", subscribed_topic, "/point_cloud");
    nh_private.param<std::string>("published_topic", published_topic, "cloud_projected");

    // Create Subscriber and listen /cloud_cleaned topic
    ros::Subscriber sub = n.subscribe<sensor_msgs::PointCloud2>(subscribed_topic, 1, cloud_cb);

    // Create Publisher
    pub = n.advertise<sensor_msgs::PointCloud2>(published_topic, 1);

    // Spin
    ros::spin();
    return 0;
}
