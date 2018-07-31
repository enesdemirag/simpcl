// Planar Segmentation
// http://www.pointclouds.org/documentation/tutorials/planar_segmentation.php

// Import dependencies
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>

// Definitions
ros::Publisher pub;

// callback
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    // Create point cloud objects for cleaned_cloud and segmented_cloud
    pcl::PointCloud<pcl::PointXYZ> segmented_cloud;
    pcl::fromROSMsg(*cloud_msg, segmented_cloud);

    // Perform the Segmentation
    pcl::ModelCoefficients coefficients;
    pcl::PointIndices inliers;
    pcl::SACSegmentation<pcl::PointXYZ> seg; // Create the segmentation object
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC); // Robust Estimator http://en.wikipedia.org/wiki/RANSAC
    seg.setDistanceThreshold(0.01); // Determine how close a point must be to the model in order to be considered an inlier
    seg.setInputCloud(segmented_cloud.makeShared());
    seg.segment(inliers, coefficients);

    // Publish the model coefficients
    pcl_msgs::ModelCoefficients ros_coefficients;
    pcl_conversions::fromPCL(coefficients, ros_coefficients);
    pub.publish(ros_coefficients);
}

// main
int main(int argc, char **argv)
{
  // Initialize ROS
  ros::init(argc, argv, "planar_segmentation");
  ros::NodeHandle n;

  // Create Subscriber and listen /cloud_cleaned topic
  ros::Subscriber sub = n.subscribe("/cloud_cleaned", 1, cloud_cb);

  // Create Publisher
  pub = n.advertise<pcl_msgs::ModelCoefficients>("planar_segmentation_result", 1);

  // Spin
  ros::spin();
  return 0;
}
