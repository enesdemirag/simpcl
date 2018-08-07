// region Dependencies and Definitions
#include <string>
#include <ros/ros.h>
#include <iostream>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
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
#include <pcl/io/vtk_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/don.h>
#ifdef PCL_ONLY_CORE_POINT_TYPES
#include <pcl/features/impl/normal_3d_omp.hpp>
#include <pcl/segmentation/impl/extract_clusters.hpp>
#endif
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
pcl::PCLPointCloud2 blob;
// endregion

// region callback
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{

}
// endregion

// region main
int main(int argc, char **argv)
{
    return 0;
}
// endregion
