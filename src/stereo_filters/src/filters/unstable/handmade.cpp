// Iterative Closest Point Algorithm
// FIXME: Publishing nothing
// ERROR:
// Import dependencies
#include <ros/ros.h>
#include <string>
#include <iostream>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

// Definitions
ros::Publisher pub;
std::string subscribed_topic;
std::string published_topic;
double threshold;
int count;

pcl::PCLPointCloud2* carrier = new pcl::PCLPointCloud2;
pcl::PCLPointCloud2ConstPtr cloudPtrCarry(carrier);

pcl::PointCloud<pcl::PointXYZ> *cloudXYZ_one = new pcl::PointCloud<pcl::PointXYZ>;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZPtr1(cloudXYZ_one);

pcl::PointCloud<pcl::PointXYZ> *cloudXYZ_two = new pcl::PointCloud<pcl::PointXYZ>;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZPtr2(cloudXYZ_two);

pcl::PointCloud<pcl::PointXYZ> *cloud_final = new pcl::PointCloud<pcl::PointXYZ>;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtrFinal(cloud_final);

// callback
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    *cloudXYZPtr1 = *cloudXYZPtr2;

    pcl_conversions::toPCL(*cloud_msg, *carrier);
    pcl::fromPCLPointCloud2(*carrier, *cloudXYZPtr2);
    std::cout << "Cloud1: " << cloudXYZ_one->width
              << "\tCloud2: " << cloudXYZ_two->width  << "\n" << std::endl;
}

// main
int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv, "handmade");
    ros::NodeHandle n;
    ros::Rate r(10);
    // Load parameters from launch file
    ros::NodeHandle nh_private("~");
    nh_private.param<std::string>("subscribed_topic", subscribed_topic, "/cloud_filtered");
    nh_private.param<std::string>("published_topic", published_topic, "cloud_final");
    nh_private.param<double>("threshold", threshold, 5000.0);
    // Create Subscriber and listen subscribed_topic
    ros::Subscriber sub = n.subscribe<sensor_msgs::PointCloud2>(subscribed_topic, 64, cloud_cb);

    // Create Publisher
    pub = n.advertise<sensor_msgs::PointCloud2>(published_topic, 64);

    while(ros::ok())
    {
        if(cloudXYZ_one->points.size() < cloudXYZ_two->points.size())
        {
            count = cloudXYZ_one->points.size();
        }
        else
        {
            count = cloudXYZ_two->points.size();
        }
        for (size_t i = -1; i < count; ++i)
        {
            if(abs(cloudXYZ_one->points[i].x - cloudXYZ_two->points[i].x) < threshold)
            {
                cloud_final->points[i].x = (cloudXYZ_one->points[i].x + cloudXYZ_two->points[i].x) / 2;
            }
            else
            {
                cloud_final->points[i].x = 0;
            }
            if(abs(cloudXYZ_one->points[i].y - cloudXYZ_two->points[i].y) < threshold)
            {
                cloud_final->points[i].y = (cloudXYZ_one->points[i].y + cloudXYZ_two->points[i].y) / 2;
            }
            else
            {
                cloud_final->points[i].y = 0;
            }
            if(abs(cloudXYZ_one->points[i].z - cloudXYZ_two->points[i].z) < threshold)
            {
                cloud_final->points[i].z = (cloudXYZ_one->points[i].z + cloudXYZ_two->points[i].z) / 2;
            }
            else
            {
                cloud_final->points[i].z = 0;
            }
        }
        cloud_final->header.frame_id = "zed_left_camera";
        cloud_final->header.stamp = ros::Time::now();

        // pcl::toPCLPointCloud2(*cloud_final, *carrier);
        pub.publish(*cloud_final); // Publish
        r.sleep();
        ros::spinOnce();
    }
    return 0;
}
