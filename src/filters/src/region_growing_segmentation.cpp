// region Dependencies
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
#include <pcl/filters/passthrough.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/region_growing_rgb.h>
// endregion

// region callback
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    pcl::search::Search <pcl::PointXYZRGB>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGB> > (new pcl::search::KdTree<pcl::PointXYZRGB>);
    pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud <pcl::PointXYZRGB>);

    pcl::IndicesPtr indices (new std::vector <int>);
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.0, 1.0);
    pass.filter (*indices);

    pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
    reg.setInputCloud (cloud);
    reg.setIndices (indices);
    reg.setSearchMethod (tree);
    reg.setDistanceThreshold (10);
    reg.setPointColorThreshold (6);
    reg.setRegionColorThreshold (5);
    reg.setMinClusterSize (600);

  std::vector <pcl::PointIndices> clusters;
  reg.extract (clusters);

  pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
}
// endregion

// region main
int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv, "region_growing_segmentation");
    ros::NodeHandle n;

    // Load parameters from launch file
    ros::NodeHandle nh_private("~");
    nh_private.param<std::string>("subscribed_topic", subscribed_topic, "/cloud_filtered");
    nh_private.param<std::string>("published_topic", published_topic, "cloud_segmented");
    nh_private.param<double>("leaf_size_x", leaf_size_x, 0.05);
    nh_private.param<double>("leaf_size_y", leaf_size_y, 0.05);
    nh_private.param<double>("leaf_size_z", leaf_size_z, 0.15);

    // Create Subscriber and listen /zed/point_cloud/cloud_registered topic
    ros::Subscriber sub = n.subscribe<sensor_msgs::PointCloud2>(subscribed_topic, 1, cloud_cb);

    // Create Publisher
    pub = n.advertise<sensor_msgs::PointCloud2>(published_topic, 1);

    // Spin
    ros::spin();
    return 0;
}
// endregion
