#include <string>
#include <ros/ros.h>
#include <iostream>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/point_operators.h>
#include <pcl/common/io.h>
#include <pcl/search/organized.h>
#include <pcl/search/octree.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/don.h>
#include <pcl/features/impl/normal_3d_omp.hpp>
#include <pcl/segmentation/impl/extract_clusters.hpp>

using namespace pcl;
using namespace std;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointNormal PointNT;
typedef pcl::PointNormal PointOutT;
typedef pcl::search::Search<PointT>::Ptr SearchPtr;

ros::Publisher pub;
std::string subscribed_topic;
std::string published_topic;
double scale1 = 0.2;
double scale2 = 2.0;
double threshold = 0.25;
double segradius = 0.2;
bool approx = false;
double decimation = 100;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    pcl::PCLPointCloud2* filtered_cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudFilteredPtr(filtered_cloud);

    pcl_conversions::toPCL(*cloud_msg, *filtered_cloud);

    pcl::PointCloud<pcl::PointXYZRGB> *xyz_cloud = new pcl::PointCloud<pcl::PointXYZRGB>;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzCloudPtr(xyz_cloud);

    // Convert the pcl::PointCloud2 type to pcl::PointCloud<pcl::PointXYZRGB>
    pcl::fromPCLPointCloud2(*cloudFilteredPtr, *xyzCloudPtr);

    // Create a search tree, use KDTreee for non-organized data.
    pcl::search::Search<PointXYZRGB>::Ptr tree;
    if (xyz_cloud->isOrganized ())
    {
        tree.reset (new pcl::search::OrganizedNeighbor<PointXYZRGB> ());
    }
    else
    {
        tree.reset (new pcl::search::KdTree<PointXYZRGB> (false));
    }

    // Set the input pointcloud for the search tree
    tree->setInputCloud (xyzCloudPtr);

    // Compute normals using both small and large scales at each point
    pcl::NormalEstimationOMP<PointXYZRGB, PointNormal> ne;
    ne.setInputCloud (xyzCloudPtr);
    ne.setSearchMethod (tree);

    ne.setViewPoint (std::numeric_limits<float>::max (), std::numeric_limits<float>::max (), std::numeric_limits<float>::max ());

    // calculate normals with the small scale
    cout << "Calculating normals for scale..." << scale1 << endl;
    pcl::PointCloud<PointNormal>::Ptr normals_small_scale (new pcl::PointCloud<PointNormal>);

    ne.setRadiusSearch (scale1);
    ne.compute (*normals_small_scale);

    // calculate normals with the large scale
    cout << "Calculating normals for scale..." << scale2 << endl;
    pcl::PointCloud<PointNormal>::Ptr normals_large_scale (new pcl::PointCloud<PointNormal>);

    ne.setRadiusSearch (scale2);
    ne.compute (*normals_large_scale);

    // Create output cloud for DoN results
    PointCloud<PointNormal>::Ptr doncloud (new pcl::PointCloud<PointNormal>);
    copyPointCloud<PointXYZRGB, PointNormal>(*xyzCloudPtr, *doncloud);

    cout << "Calculating DoN... " << endl;
    // Create DoN operator
    pcl::DifferenceOfNormalsEstimation<PointXYZRGB, PointNormal, PointNormal> don;
    don.setInputCloud (xyzCloudPtr);
    don.setNormalScaleLarge (normals_large_scale);
    don.setNormalScaleSmall (normals_small_scale);

    // Compute DoN
    don.computeFeature (*doncloud);
    //pub.publish(*doncloud);

    // Filter by magnitude
    cout << "Filtering out DoN mag <= " << threshold << "..." << endl;

    // Build the condition for filtering
    pcl::ConditionOr<PointNormal>::Ptr range_cond (new pcl::ConditionOr<PointNormal> ());
    range_cond->addComparison (pcl::FieldComparison<PointNormal>::ConstPtr (new pcl::FieldComparison<PointNormal> ("curvature", pcl::ComparisonOps::GT, threshold)));
    // Build the filter
    pcl::ConditionalRemoval<PointNormal> condrem (range_cond);
    condrem.setInputCloud (doncloud);

    pcl::PointCloud<PointNormal>::Ptr doncloud_filtered (new pcl::PointCloud<PointNormal>);

    // Apply filter
    condrem.filter (*doncloud_filtered);

    doncloud = doncloud_filtered;

    // Save filtered output
    std::cout << "Filtered Pointcloud: " << doncloud->points.size () << " data points." << std::endl;

    pub.publish(*doncloud);

    // Filter by magnitude
    cout << "Clustering using EuclideanClusterExtraction with tolerance <= " << segradius << "..." << endl;

    pcl::search::KdTree<PointNormal>::Ptr segtree (new pcl::search::KdTree<PointNormal>);
    segtree->setInputCloud (doncloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointNormal> ec;

    ec.setClusterTolerance (segradius);
    ec.setMinClusterSize (50);
    ec.setMaxClusterSize (100000);
    ec.setSearchMethod (segtree);
    ec.setInputCloud (doncloud);
    ec.extract (cluster_indices);

    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it, j++)
    {
        pcl::PointCloud<PointNormal>::Ptr cloud_cluster_don (new pcl::PointCloud<PointNormal>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        {
            cloud_cluster_don->points.push_back (doncloud->points[*pit]);
        }

        cloud_cluster_don->width = int (cloud_cluster_don->points.size ());
        cloud_cluster_don->height = 1;
        cloud_cluster_don->is_dense = true;
        //pub.publish(*cloud_cluster_don);
    }
}

int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv, "don_segmentation");
    ros::NodeHandle n;

    // Load parameters from launch file
    ros::NodeHandle nh_private("~");
    nh_private.param<std::string>("subscribed_topic", subscribed_topic, "/cloud_filtered");
    nh_private.param<std::string>("published_topic", published_topic, "cloud_segmented");

    // Create Subscriber and listen subscribed_topic
    ros::Subscriber sub = n.subscribe(subscribed_topic, 1, cloud_cb);

    // Create Publisher
    pub = n.advertise<pcl::PointCloud<pcl::PointXYZRGB> >(published_topic, 1);

    // Spin
    ros::spin();
    return 0;
}
