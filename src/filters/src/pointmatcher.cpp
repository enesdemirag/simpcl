//FIXME: Fix conversation functions
// https://github.com/ethz-asl/libpointmatcher/blob/master/examples/icp.cpp

// Import dependencies
#include <cassert>
#include <iostream>
#include <fstream>
#include "tf/tf.h"
#include "ros/ros.h"
#include "pointmatcher/PointMatcher.h"
#include "pointmatcher_ros/point_cloud.h"
#include "pointmatcher_ros/transform.h"

using namespace std;
using namespace PointMatcherSupport;

// Definitions
ros::Publisher pub;
std::string subscribed_topic;
std::string published_topic;
typedef PointMatcher<float> PM;
typedef PM::DataPoints DP;
DP cloud1;
DP cloud2;

// callback
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    cloud2 = cloud1;
    cloud1 = PointMatcher_ros::rosMsgToPointMatcherCloud(*cloud_msg);
}

// main
int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv, "pointmatcher");
    ros::NodeHandle n;
    ros::Rate r(10);

    // Load parameters from launch file
    ros::NodeHandle nh_private("~");
    nh_private.param<std::string>("subscribed_topic", subscribed_topic, "/point_cloud/cloud_registered");
    nh_private.param<std::string>("published_topic", published_topic, "cloud_downsampled");

    // Create Subscriber and listen /zed/point_cloud/cloud_registered topic
    ros::Subscriber sub = n.subscribe<sensor_msgs::PointCloud2>(subscribed_topic, 1, cloud_cb);

    // Create Publisher
    pub = n.advertise<sensor_msgs::PointCloud2>(published_topic, 1);

    while(ros::ok())
    {
        PM::ICP icp;
    	PM::TransformationParameters translation = parseTranslation(initTranslation, cloudDimension);
    	PM::TransformationParameters rotation =	parseRotation(initRotation, cloudDimension);
    	PM::TransformationParameters initTransfo = translation*rotation;
    	PM::Transformation* rigidTrans;
    	rigidTrans = PM::get().REG(Transformation).create("RigidTransformation");
    	if (!rigidTrans->checkParameters(initTransfo))
        {
    		cerr << endl
    			 << "Initial transformation is not rigid, identiy will be used"
    			 << endl;
    		initTransfo = PM::TransformationParameters::Identity(cloudDimension + 1, cloudDimension + 1);
    	}
    	const DP initializedData = rigidTrans->compute(cloud1, initTransfo);

    	PM::TransformationParameters T = icp(initializedData, cloud2);
    	if(isVerbose)
    		cout << "match ratio: " << icp.errorMinimizer->getWeightedPointUsedRatio() << endl;

    	DP cloud_final(initializedData);
    	icp.transformations.apply(cloud_final, T);

        sensor_msgs::PointCloud2 output_cloud;
        output_cloud = PointMatcher_ros::pointMatcherCloudToRosMsg(cloud_final);
        pub.publish(output_cloud);
        r.sleep();
        ros::spinOnce();
    }
    return 0;
}
