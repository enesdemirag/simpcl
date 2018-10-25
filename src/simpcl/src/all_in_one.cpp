// Import dependencies
#include <ros/ros.h>
#include <string>
#include <iostream>
// PCL
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
// Filters
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/registration/icp.h>

// Definitions
ros::Publisher pub;
std::string subscribed_topic;
std::string published_topic;
int loop_rate;
int buffer_size;

double leaf_size_x, leaf_size_y, leaf_size_z; // Voxel Filter parameters

double min_value_x, max_value_x, // PassThrough Filter parameters
       min_value_y, max_value_y,
       min_value_z, max_value_z;

int meanK; // Statistical Outlier Removal Filter parameters
double mulThresh;

double max_distance; // ICP parameters
int max_iteration;

// Point CLoud Objects
pcl::PCLPointCloud2* raw_cloud = new pcl::PCLPointCloud2;
pcl::PCLPointCloud2ConstPtr cloudPtr1(raw_cloud);
pcl::PCLPointCloud2 carrier_cloud;
sensor_msgs::PointCloud2 carrier;

pcl::PointCloud<pcl::PointXYZ> *cloudXYZ_one = new pcl::PointCloud<pcl::PointXYZ>;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZPtr1(cloudXYZ_one);

pcl::PointCloud<pcl::PointXYZ> *cloudXYZ_two = new pcl::PointCloud<pcl::PointXYZ>;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZPtr2(cloudXYZ_two);

pcl::PointCloud<pcl::PointXYZ> *cloud_icp = new pcl::PointCloud<pcl::PointXYZ>;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtrFinal(cloud_icp);

// callback function
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    pcl_conversions::toPCL(*cloud_msg, *raw_cloud); // Convert to PCL data type

    // Perform the PassThrough Filter to the X axis
    pcl::PassThrough<pcl::PCLPointCloud2> px_filter;
    px_filter.setInputCloud(cloudPtr1); // Pass filtered_cloud to the filter
    px_filter.setFilterFieldName("x"); // Set axis x
    px_filter.setFilterLimits(min_value_x, max_value_x); // Set limits min_value to max_value
    px_filter.filter(carrier_cloud); // Restore output data in second_cloud

    pcl_conversions::moveFromPCL(carrier_cloud, carrier); // Convert to ROS data type
    pcl::PCLPointCloud2* x_removed_cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr_X(x_removed_cloud);
    pcl_conversions::toPCL(carrier, *x_removed_cloud); // Convert to PCL data type

    // Perform the PassThrough Filter to the Y axis
    pcl::PassThrough<pcl::PCLPointCloud2> py_filter;
    py_filter.setInputCloud(cloudPtr_X); // Pass filtered_cloud to the filter
    py_filter.setFilterFieldName("y"); // Set axis y
    py_filter.setFilterLimits(min_value_y, max_value_y); // Set limits min_value to max_value
    py_filter.filter(carrier_cloud); // Restore output data in second_cloud

    pcl_conversions::moveFromPCL(carrier_cloud, carrier); // Convert to ROS data type
    pcl::PCLPointCloud2* y_removed_cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr_Y(y_removed_cloud);
    pcl_conversions::toPCL(carrier, *y_removed_cloud); // Convert to PCL data type

    // Perform the PassThrough Filter to the Z axis
    pcl::PassThrough<pcl::PCLPointCloud2> pz_filter;
    pz_filter.setInputCloud(cloudPtr_Y); // Pass filtered_cloud to the filter
    pz_filter.setFilterFieldName("z"); // Set axis z
    pz_filter.setFilterLimits(min_value_z, max_value_z); // Set limits min_value to max_value
    pz_filter.filter(carrier_cloud); // Restore output data in second_cloud

    pcl_conversions::moveFromPCL(carrier_cloud, carrier); // Convert to ROS data type
    pcl::PCLPointCloud2* pass_cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr2(pass_cloud);
    pcl_conversions::toPCL(carrier, *pass_cloud); // Convert to PCL data type

    // Perform the VoxelGrid Filter
    pcl::VoxelGrid<pcl::PCLPointCloud2> v_filter;
    v_filter.setInputCloud(cloudPtr2); // Pass raw_cloud to the filter
    v_filter.setLeafSize(leaf_size_x, leaf_size_y, leaf_size_z); // Set leaf size
    v_filter.filter(carrier_cloud); // Store output data in first_cloud

    pcl_conversions::moveFromPCL(carrier_cloud, carrier); // Convert to ROS data type
    pcl::PCLPointCloud2* voxel_cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr3(voxel_cloud);
    pcl_conversions::toPCL(carrier, *voxel_cloud); // Convert to PCL data type

    // Perform the Statistical Outlier Removal Filter
    pcl::StatisticalOutlierRemoval<pcl::PCLPointCloud2> s_filter;
    s_filter.setInputCloud(cloudPtr3); // Pass filtered_cloud to the filter
    s_filter.setMeanK(meanK); // Set the number of neighbors to analyze for each point
    s_filter.setStddevMulThresh(mulThresh); // Set standard deviation multiplier
    s_filter.filter(carrier_cloud); // Restore output data in filtered_cloud

    *cloudXYZPtr1 = *cloudXYZPtr2;
    pcl::fromPCLPointCloud2(carrier_cloud, *cloudXYZPtr2);
}

// main function
int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv, "all_in_one");
    ros::NodeHandle n;

    // Load parameters from launch file
    ros::NodeHandle nh_private("~");
    // ROS parameters
    nh_private.param<std::string>("subscribed_topic", subscribed_topic, "/point_cloud");
    nh_private.param<std::string>("published_topic", published_topic, "cloud_filtered");
    nh_private.param<int>("loop_rate", loop_rate, 10);
    nh_private.param<int>("buffer_size", buffer_size, 1);
    // Voxel Filter Parameters
    nh_private.param<double>("leaf_size_x", leaf_size_x, 0.12);
    nh_private.param<double>("leaf_size_y", leaf_size_y, 0.12);
    nh_private.param<double>("leaf_size_z", leaf_size_z, 0.12);
    // PassThrough Filter Parameters
    nh_private.param<double>("min_value_x", min_value_x, 0.5);
    nh_private.param<double>("max_value_x", max_value_x, 10.0);
    nh_private.param<double>("min_value_y", min_value_y, -20.0);
    nh_private.param<double>("max_value_y", max_value_y, 20.0);
    nh_private.param<double>("min_value_z", min_value_z, -20.0);
    nh_private.param<double>("max_value_z", max_value_z, 20.0);
    // Statistical Outlier Removal Filter Parameters
    nh_private.param<int>("meanK", meanK, 16);
    nh_private.param<double>("mulThresh", mulThresh, 0.001);
    // ICP Parameters
    nh_private.param<double>("max_distance", max_distance, 0.5);
    nh_private.param<int>("max_iteration", max_iteration, 16);

    ros::Rate r(loop_rate);
    // Create Subscriber and listen subscribed_topic
    ros::Subscriber sub = n.subscribe<sensor_msgs::PointCloud2>(subscribed_topic, buffer_size, cloud_cb);

    // Create Publisher
    pub = n.advertise<sensor_msgs::PointCloud2>(published_topic, buffer_size);

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
