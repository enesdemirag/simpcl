// Planar Segmentation
// http://www.pointclouds.org/documentation/tutorials/planar_segmentation.php

// Import dependencies
#include <ros/ros.h>
#include <string>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl_ros/point_cloud.h>

// Definitions
ros::Publisher pub;
std::string subscribed_topic;
std::string published_topic;
double distanceThreshold;
bool optimizeCoefficients;

// callback
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    // Container for original & filtered data
    pcl::PCLPointCloud2* filtered_cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudFilteredPtr(filtered_cloud);

    pcl_conversions::toPCL(*cloud_msg, *filtered_cloud);

    pcl::PointCloud<pcl::PointXYZRGB> *xyz_cloud = new pcl::PointCloud<pcl::PointXYZRGB>;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzCloudPtr(xyz_cloud);

    // Convert the pcl::PointCloud2 type to pcl::PointCloud<pcl::PointXYZRGB>
    pcl::fromPCLPointCloud2(*cloudFilteredPtr, *xyzCloudPtr);

    // Create a point cloud object to hold the RANSAC result
    pcl::PointCloud<pcl::PointXYZRGB> *xyz_cloud_ransac_filtered = new pcl::PointCloud<pcl::PointXYZRGB>;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzCloudPtrRansacFiltered(xyz_cloud_ransac_filtered);

    // Perform RANSAC Planar Filtration to remove ground
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZRGB> seg1;
    seg1.setOptimizeCoefficients(optimizeCoefficients);
    seg1.setModelType(pcl::SACMODEL_PLANE);
    seg1.setMethodType(pcl::SAC_RANSAC);
    seg1.setMaxIterations(50);
    seg1.setDistanceThreshold(distanceThreshold);
    seg1.setInputCloud(xyzCloudPtr);
    seg1.segment(*inliers, *coefficients);

    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud(xyzCloudPtr);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*xyzCloudPtrRansacFiltered);

    // Publish
    pub.publish(*xyz_cloud_ransac_filtered);
}

// main
int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv, "planar_segmentation");
    ros::NodeHandle n;

    // Load parameters from launch file
    ros::NodeHandle nh_private("~");
    nh_private.param<std::string>("subscribed_topic", subscribed_topic, "/cloud_filtered");
    nh_private.param<std::string>("published_topic", published_topic, "cloud_segmented");
    nh_private.param<bool>("optimizeCoefficients", optimizeCoefficients, true);
    nh_private.param<double>("distanceThreshold", distanceThreshold, 0.3);

    // Create Subscriber and listen subscribed_topic
    ros::Subscriber sub = n.subscribe(subscribed_topic, 48, cloud_cb);

    // Create Publisher
    pub = n.advertise<pcl::PointCloud<pcl::PointXYZRGB> >(published_topic, 48);

    // Spin
    ros::spin();
    return 0;
}
