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
#include <pcl/filters/conditional_removal.h> //and the other usuals

// Definitions
ros::Publisher pub;
int use_hsi;
double r_min_value, r_max_value, g_min_value, g_max_value, b_min_value, b_max_value;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*cloud_msg, *rgb_cloud);
    pcl::PCLPointCloud2 cloud_filtered_ros;

    pcl::ConditionalRemoval<pcl::PointXYZRGB> color_filter;

    pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr color_cond (new pcl::ConditionAnd<pcl::PointXYZRGB> ());

    if (use_hsi)
    {
        pcl::PackedHSIComparison<pcl::PointXYZRGB>::Ptr r_low(new pcl::PackedHSIComparison<pcl::PointXYZRGB>("h", pcl::ComparisonOps::GE, r_min_value));
        pcl::PackedHSIComparison<pcl::PointXYZRGB>::Ptr r_high(new pcl::PackedHSIComparison<pcl::PointXYZRGB>("h", pcl::ComparisonOps::LE, r_max_value));

        pcl::PackedHSIComparison<pcl::PointXYZRGB>::Ptr g_low(new pcl::PackedHSIComparison<pcl::PointXYZRGB>("s", pcl::ComparisonOps::GE, g_min_value));
        pcl::PackedHSIComparison<pcl::PointXYZRGB>::Ptr g_high(new pcl::PackedHSIComparison<pcl::PointXYZRGB>("s", pcl::ComparisonOps::LE, g_max_value));

        pcl::PackedHSIComparison<pcl::PointXYZRGB>::Ptr b_low(new pcl::PackedHSIComparison<pcl::PointXYZRGB>("i", pcl::ComparisonOps::GE, b_min_value));
        pcl::PackedHSIComparison<pcl::PointXYZRGB>::Ptr b_high(new pcl::PackedHSIComparison<pcl::PointXYZRGB>("i", pcl::ComparisonOps::LE, b_max_value));

        color_cond->addComparison (r_low);
        color_cond->addComparison (r_high);

        color_cond->addComparison (g_low);
        color_cond->addComparison (g_high);

        color_cond->addComparison (b_low);
        color_cond->addComparison (b_high);
    }
    else
    {
        pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr r_low(new pcl::PackedRGBComparison<pcl::PointXYZRGB>("r", pcl::ComparisonOps::GE, r_min_value));
        pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr r_high(new pcl::PackedRGBComparison<pcl::PointXYZRGB>("r", pcl::ComparisonOps::LE, r_max_value));

        pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr g_low(new pcl::PackedRGBComparison<pcl::PointXYZRGB>("g", pcl::ComparisonOps::GE, g_min_value));
        pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr g_high(new pcl::PackedRGBComparison<pcl::PointXYZRGB>("g", pcl::ComparisonOps::LE, g_max_value));

        pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr b_low(new pcl::PackedRGBComparison<pcl::PointXYZRGB>("b", pcl::ComparisonOps::GE, b_min_value));
        pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr b_high(new pcl::PackedRGBComparison<pcl::PointXYZRGB>("b", pcl::ComparisonOps::LE, b_max_value));    
        
        color_cond->addComparison (r_low);
        color_cond->addComparison (r_high);

        color_cond->addComparison (g_low);
        color_cond->addComparison (g_high);

        color_cond->addComparison (b_low);
        color_cond->addComparison (b_high);
    }    


    // Build the filter
    color_filter.setInputCloud(rgb_cloud);
    color_filter.setCondition (color_cond);
    color_filter.filter(*cloud_filtered);

    pcl::toPCLPointCloud2(*cloud_filtered, cloud_filtered_ros);

    pub.publish(cloud_filtered_ros);
}

int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv, "color_filter");
    ros::NodeHandle n;

    // Load parameters from launch file
    ros::NodeHandle nh_private("~");

    nh_private.param<int>("use_hsi", use_hsi, 0);

    nh_private.param<double>("r_min", r_min_value, 0.5); // 50 cm
    nh_private.param<double>("r_max", r_max_value, 15.0); // 15 meters

    nh_private.param<double>("g_min", g_min_value, 0.5); // 50 cm
    nh_private.param<double>("g_max", g_max_value, 15.0); // 15 meters

    nh_private.param<double>("b_min", b_min_value, 0.5); // 50 cm
    nh_private.param<double>("b_max", b_max_value, 15.0); // 15 meters

    if (use_hsi)
    {
        ROS_WARN_STREAM("Using HSI instead of RGB, use the [r/g/b]_[min/max]_value node parameters as H, S, I respectively.");
    }

    // Create Subscriber and listen /cloud_downsampled topic
    ros::Subscriber sub = n.subscribe<sensor_msgs::PointCloud2>("points_in", 1, cloud_cb);

    // Create Publisher
    pub = n.advertise<sensor_msgs::PointCloud2>("points_out", 1);

    // Spin
    ros::spin();
    return 0;
}
