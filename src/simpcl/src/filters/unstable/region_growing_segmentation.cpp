// Color-based Region Growing Segmentation for removing the ground
// http://pointclouds.org/documentation/tutorials/region_growing_rgb_segmentation.php
// TODO: Fix "Assertion `px != 0' failed. Aborted (core dumped)" error

// Dependencies
#include <iostream>
#include <vector>
#include <string>
#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/region_growing.h>

// Definitions
ros::Publisher pub;
std::string subscribed_topic;
std::string published_topic;

// callback
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    pcl::search::Search <pcl::PointXYZRGB>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGB> > (new pcl::search::KdTree<pcl::PointXYZRGB>);
    pcl::PointCloud <pcl::PointXYZRGB>::Ptr raw_cloud (new pcl::PointCloud <pcl::PointXYZRGB>);
    pcl::IndicesPtr indices(new std::vector <int>);
    pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);

    // Generate Normals
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod(tree);
    normal_estimator.setInputCloud(raw_cloud);
    normal_estimator.setKSearch(50);
    normal_estimator.compute(*normals);

    // Perform Region Growing Segmentation
    pcl::RegionGrowing<pcl::PointXYZRGB, pcl::Normal> reg;
    reg.setMinClusterSize(50);
    reg.setMaxClusterSize(1000000);
    reg.setSearchMethod(tree);
    reg.setNumberOfNeighbours(30);
    reg.setInputCloud(raw_cloud);
    reg.setInputNormals(normals);
    reg.setSmoothnessThreshold(3.0 / 180.0 * M_PI);
    reg.setCurvatureThreshold(1.0);

    pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud (new pcl::PointCloud <pcl::PointXYZRGB>);
    colored_cloud = reg.getColoredCloud();

    /*sensor_msgs::PointCloud2 cloud_out;
    pcl::toROSMsg(*colored_cloud, cloud_out);*/
    pub.publish(colored_cloud);
}

// main
int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv, "region_growing_segmentation");
    ros::NodeHandle n;

    // Load parameters from launch file
    ros::NodeHandle nh_private("~");
    nh_private.param<std::string>("subscribed_topic", subscribed_topic, "/point_cloud");
    nh_private.param<std::string>("published_topic", published_topic, "cloud_segmented");

    // Create Subscriber and listen to subscribed_topic
    ros::Subscriber sub = n.subscribe<sensor_msgs::PointCloud2>(subscribed_topic, 1, cloud_cb);

    // Create Publisher
    pub = n.advertise<pcl::PointCloud<pcl::PointXYZRGB> >(published_topic, 1); //<sensor_msgs::PointCloud2>

    // Spin
    ros::spin();
    return 0;
}
