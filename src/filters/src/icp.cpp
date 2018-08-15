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

pcl::PointCloud<pcl::PointXYZ> *cloudXYZ_three = new pcl::PointCloud<pcl::PointXYZ>;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZPtr3(cloudXYZ_three);

pcl::PointCloud<pcl::PointXYZ> *cloudXYZ_four = new pcl::PointCloud<pcl::PointXYZ>;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZPtr4(cloudXYZ_four);

pcl::PointCloud<pcl::PointXYZ> *cloud_icp1 = new pcl::PointCloud<pcl::PointXYZ>;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtrICP1(cloud_icp1);

pcl::PointCloud<pcl::PointXYZ> *cloud_icp2 = new pcl::PointCloud<pcl::PointXYZ>;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtrICP2(cloud_icp2);

pcl::PointCloud<pcl::PointXYZ> *cloud_final = new pcl::PointCloud<pcl::PointXYZ>;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtrFinal(cloud_final);

// callback
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    *cloudXYZPtr1 = *cloudXYZPtr2;
    *cloudXYZPtr2 = *cloudXYZPtr3;
    *cloudXYZPtr3 = *cloudXYZPtr4;

    pcl_conversions::toPCL(*cloud_msg, *carrier);
    pcl::fromPCLPointCloud2(*carrier, *cloudXYZPtr4);
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
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp1;
        icp1.setInputSource(cloudXYZPtr1);
        icp1.setInputTarget(cloudXYZPtr2);
        icp1.setMaxCorrespondenceDistance(0.1); // Set the max correspondence distance to 10cm
        icp1.setMaximumIterations(32); // Set the maximum number of iterations (criterion 1)
        icp1.setTransformationEpsilon(1e-8); // Set the transformation epsilon (criterion 2)
        icp1.setEuclideanFitnessEpsilon(1); // Set the euclidean distance difference epsilon (criterion 3)
        icp1.align(*cloud_icp1); // Perform the alignment

        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp2;
        icp2.setInputSource(cloudXYZPtr3);
        icp2.setInputTarget(cloudXYZPtr4);
        icp2.setMaxCorrespondenceDistance(0.1); // Set the max correspondence distance to 10cm
        icp2.setMaximumIterations(32); // Set the maximum number of iterations (criterion 1)
        icp2.setTransformationEpsilon(1e-8); // Set the transformation epsilon (criterion 2)
        icp2.setEuclideanFitnessEpsilon(1); // Set the euclidean distance difference epsilon (criterion 3)
        icp2.align(*cloud_icp2); // Perform the alignment

        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setInputSource(cloudXYZPtr1);
        icp.setInputTarget(cloudXYZPtr2);
        icp.setMaxCorrespondenceDistance(0.025); // Set the max correspondence distance to 2.5cm
        icp.setMaximumIterations(64); // Set the maximum number of iterations (criterion 1)
        icp.setTransformationEpsilon(1e-8); // Set the transformation epsilon (criterion 2)
        icp.setEuclideanFitnessEpsilon(1); // Set the euclidean distance difference epsilon (criterion 3)
        icp.align(*cloud_final); // Perform the alignment

        pub.publish(*cloud_final); // Publish
        r.sleep();
        ros::spinOnce();
    }
    return 0;
}
