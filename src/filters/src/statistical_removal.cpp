// Remove noise with "Statistical Outlier Removal Filter"

// Import dependencies
#include <ros/ros.h>
#include <iostream>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <sensor_msgs/PointCloud2.h>

// Definitions
typedef pcl::PCLPointCloud2 PC2;

// callback
void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& passed_cloud)
{
    //Create point cloud object for cleaned_cloud
    PC2::Ptr cleaned_cloud (new PC2);
    cleaned_cloud->header.frame_id = "cleaned_tf_frame";
    cleaned_cloud->height = cleaned_cloud->width = 1;

    // Create the Statistical Outlier Removal Filter
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> s_filter;
    s_filter.setInputCloud (passed_cloud); // Pass passed_cloud to the filter
    s_filter.setMeanK (50); //
    s_filter.setStddevMulThresh (1.0); //
    s_filter.filter (*cleaned_cloud); // Store output data in cleaned_cloud
}

// main
int main(int argc, char** argv[])
{
    // Create node
    ros::init(argc, argv, "statistical_removal");
    ros::NodeHandle n;
    ros::Rate loop_rate(60);

    // Create Publisher
    ros::Publisher pub = n.advertise<PC2>("/cloud_cleaned", 1);

    // Create Subscriber and listen /cloud_passed topic
    ros::Subscriber sub = n.subscribe<PC2>("/cloud_passed", 1, cloud_cb);

    pub.publish(cleaned_cloud); // Publish cleaned_cloud to the /cloud_cleaned topic

    // Kill node
    ros::spinOnce();
    loop_rate.sleep();
    return 0;
}
