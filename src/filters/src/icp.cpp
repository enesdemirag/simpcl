// Iterative Closest Point Algorithm
// Import dependencies
#include <ros/ros.h>
#include <string>
#include <iostream>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/registration/icp.h>

// Definitions
ros::Publisher pub;
std::string subscribed_topic;
std::string published_topic;
double max_distance;
int max_iteration;

pcl::PCLPointCloud2* cloud2_one = new pcl::PCLPointCloud2;
pcl::PCLPointCloud2ConstPtr cloud2Ptr1(cloud2_one);

pcl::PCLPointCloud2* cloud2_two = new pcl::PCLPointCloud2;
pcl::PCLPointCloud2ConstPtr cloud2Ptr2(cloud2_two);

pcl::PCLPointCloud2* carrier = new pcl::PCLPointCloud2;
pcl::PCLPointCloud2ConstPtr cloudPtrCarry(carrier);

pcl::PointCloud<pcl::PointXYZ> *cloudXYZ_one = new pcl::PointCloud<pcl::PointXYZ>;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZPtr1(cloudXYZ_one);

pcl::PointCloud<pcl::PointXYZ> *cloudXYZ_two = new pcl::PointCloud<pcl::PointXYZ>;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZPtr2(cloudXYZ_two);

pcl::PointCloud<pcl::PointXYZ> *cloud_icp = new pcl::PointCloud<pcl::PointXYZ>;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtrFinal(cloud_icp);

// callback
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    *cloudXYZPtr1 = *cloudXYZPtr2;

    pcl_conversions::toPCL(*cloud_msg, *carrier);
    pcl::fromPCLPointCloud2(*carrier, *cloudXYZPtr2);
}

// main
int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv, "icp");
    ros::NodeHandle n;
    ros::Rate r(10);
    // Load parameters from launch file
    ros::NodeHandle nh_private("~");
    nh_private.param<std::string>("subscribed_topic", subscribed_topic, "/cloud_filtered");
    nh_private.param<std::string>("published_topic", published_topic, "cloud_final");
    nh_private.param<double>("max_distance", max_distance, 0.1);
    nh_private.param<int>("max_iteration", max_iteration, 32);

    // Create Subscriber and listen subscribed_topic
    ros::Subscriber sub = n.subscribe<sensor_msgs::PointCloud2>(subscribed_topic, 64, cloud_cb);

    // Create Publisher
    pub = n.advertise<sensor_msgs::PointCloud2>(published_topic, 64);

    while(ros::ok())
    {
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setInputSource(cloudXYZPtr1);
        icp.setInputTarget(cloudXYZPtr2);
        icp.setMaxCorrespondenceDistance(max_distance); // Set the max correspondence distance to 10cm
        icp.setMaximumIterations(max_iteration); // Set the maximum number of iterations (criterion 1)
        icp.setTransformationEpsilon(1e-8); // Set the transformation epsilon (criterion 2)
        icp.setEuclideanFitnessEpsilon(1); // Set the euclidean distance difference epsilon (criterion 3)
        icp.align(*cloud_icp); // Perform the alignment

        pub.publish(*cloud_icp); // Publish
        r.sleep();
        ros::spinOnce();
    }
    return 0;
}
