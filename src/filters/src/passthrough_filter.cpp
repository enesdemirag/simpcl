// Getting points between 0.5m to 20m with "Passthrough Filter"

// Import dependencies
#include <ros/ros.h>
#include <iostream>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <sensor_msgs/PointCloud2.h>

// Definitions
typedef pcl::PCLPointCloud2 PC2;

// callback
void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& downsampled_cloud)
{
    //Create point cloud object for passed_cloud
    PC2::Ptr passed_cloud (new PC2);
    passed_cloud->header.frame_id = "passthrough_tf_frame";
    passed_cloud->height = passed_cloud->width = 1;

    // Create the PassThrough Filter
    pcl::PassThrough<pcl::PointXYZ> p_filter;
    p_filter.setInputCloud (downsampled_cloud); // Pass downsampled_cloud to the filter
    p_filter.setFilterFieldName ("z"); // Set axis
    p_filter.setFilterLimits (0.5, 15.0); // Set limits 0.5m to 15m
    p_filter.filter (*passed_cloud); // Store output data in passed_cloud
}

// main
int main(int argc, char** argv[])
{
    // Create node
    ros::init(argc, argv, "passthrough_filter");
    ros::NodeHandle n;
    ros::Rate loop_rate(60);

    // Create Publisher
    ros::Publisher pub = n.advertise<PC2>("/cloud_passed", 1);

    // Create Subscriber and listen /cloud_downsampled topic
    ros::Subscriber sub = n.subscribe<PC2>("/cloud_downsampled", 1, cloud_cb);

    pub.publish(passed_cloud); // Publish passed_cloud to the /cloud_passed topic

    // Kill node
    ros::spinOnce();
    loop_rate.sleep();
    return 0;
}
