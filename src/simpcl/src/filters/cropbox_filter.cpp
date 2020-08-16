// Import dependencies
#include <ros/ros.h>
#include <string>
#include <iostream>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <tf/transform_listener.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/crop_box.h>

// Definitions
ros::Publisher pub;
double x_min_value, x_max_value, y_min_value, y_max_value, z_min_value, z_max_value;

tf::StampedTransform transform;
std::string base_frame;
std::string target_frame;
int use_transform;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    sensor_msgs::PointCloud2 buffer_local;
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    
    //Create point cloud objects for downsampled_cloud and passed_cloud
    pcl::PCLPointCloud2* downsampled_cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr(downsampled_cloud);
    pcl::PCLPointCloud2 passed_cloud;

    // Convert to PCL data type
    pcl_conversions::toPCL(*cloud_msg, *downsampled_cloud);

    if (use_transform)
    {
        // TODO: use pcl::CropBox<pcl::PointXYZRGB> cbox_filter; instead
        // to eliminate above line, pcl::toPCLPointCloud2(*cloud_transformed, *downsampled_cloud);
        // Extra conversion.

        pcl::fromPCLPointCloud2(*downsampled_cloud, *temp_cloud);

        pcl_ros::transformPointCloud(*temp_cloud, *cloud_transformed, transform);
        pcl::toPCLPointCloud2(*cloud_transformed, *downsampled_cloud);
    }
    
    // CropBox Filter, 3D Box filtering.
    pcl::CropBox<pcl::PCLPointCloud2> cbox_filter;

    // Set boundaries
    cbox_filter.setMin(Eigen::Vector4f(x_min_value, y_min_value, z_min_value, 1.0));
    cbox_filter.setMax(Eigen::Vector4f(x_max_value, y_max_value, z_max_value, 1.0));

    // Set input cloud
    cbox_filter.setInputCloud(cloudPtr);
    cbox_filter.filter(passed_cloud);

    // Convert to ROS data type
    sensor_msgs::PointCloud2 output_cloud;
    pcl_conversions::moveFromPCL(passed_cloud, output_cloud);

    output_cloud.header.frame_id = "/ws_center";
    pub.publish(output_cloud);
}

int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv, "cropbox_filter");
    ros::NodeHandle n;

    // Load parameters from launch file
    ros::NodeHandle nh_private("~");

    nh_private.param<double>("x_min", x_min_value, 0.5); // 50 cm
    nh_private.param<double>("x_max", x_max_value, 15.0); // 15 meters

    nh_private.param<double>("y_min", y_min_value, 0.5); // 50 cm
    nh_private.param<double>("y_max", y_max_value, 15.0); // 15 meters

    nh_private.param<double>("z_min", z_min_value, 0.5); // 50 cm
    nh_private.param<double>("z_max", z_max_value, 15.0); // 15 meters


    nh_private.param<int>("use_transform", use_transform, 1); // 15 meters

    nh_private.param<std::string>("base_frame", base_frame, "scan");
    nh_private.param<std::string>("target_frame", target_frame, "base_link");


    if (use_transform)
    {
        ROS_WARN("The use_transform parameter is enabled, point cloud will be transformed from: %s, to: %s", base_frame.c_str(), target_frame.c_str());
        tf::TransformListener listener;
        listener.waitForTransform(target_frame, base_frame, ros::Time(0), ros::Duration(10.0) );
        try
        {
            listener.lookupTransform(target_frame, base_frame,  ros::Time(0), transform);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("TransformException: %s, BaseFrame: %s, TargetFrame: %s", ex.what(), base_frame.c_str(), target_frame.c_str());
            ros::Duration(1.0).sleep();
            // return -1;
        }
    }

    // Create Subscriber and listen /cloud_downsampled topic
    ros::Subscriber sub = n.subscribe<sensor_msgs::PointCloud2>("points_in", 1, cloud_cb);

    // Create Publisher
    pub = n.advertise<sensor_msgs::PointCloud2>("points_out", 1);

    // Spin
    ros::spin();
    return 0;
}
