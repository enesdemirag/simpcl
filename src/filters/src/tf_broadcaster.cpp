// This node broudcasts Odometry data from a base frame to child frame

// Import dependencies
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <iostream>

// Definitions
nav_msgs::Odometry odom_msg;
std::string subscribed_topic;
std::string base_frame;
std::string child_frame;
int loop_rate;

// callback
void callback(const nav_msgs::OdometryConstPtr& msg)
{
    odom_msg = *msg; // Store odometry data in odom_msg
}

// main
int main(int argc, char **argv)
{
    ros::init(argc, argv, "tf_broadcaster");
    ros::NodeHandle n;

    // Load parameters from launch file
    ros::NodeHandle nh_private("~");
    nh_private.param<std::string>("subscribed_topic", subscribed_topic, "/odom");
    nh_private.param<std::string>("base_frame", base_frame, "world");
    nh_private.param<std::string>("child_frame", child_frame, "zed_left_camera");
    nh_private.param<int>("loop_rate", loop_rate, 10);

    // Subscribe
    ros::Subscriber sub = n.subscribe<nav_msgs::Odometry>(subscribed_topic, 48, callback);
    // Create broudcaster and transformer objects
    tf::TransformBroadcaster odom_broadcaster;
    geometry_msgs::TransformStamped odom_transform;

    ros::Rate r(loop_rate); // Set loop rate

    while(ros::ok())
    {
        // Transform from one frame to the other
        // X Y Z
        odom_transform.transform.translation.x=odom_msg.pose.pose.position.x;
        odom_transform.transform.translation.y=odom_msg.pose.pose.position.y;
        odom_transform.transform.translation.z=odom_msg.pose.pose.position.z;
        // Yaw Pitch Roll
        odom_transform.transform.rotation = odom_msg.pose.pose.orientation;
        // Print coordinates
        std::cout << "(" << odom_transform.transform.translation.x << ","
                         << odom_transform.transform.translation.y << ","
                         << odom_transform.transform.translation.z << ","
                         << odom_transform.transform.rotation.x << ","
                         << odom_transform.transform.rotation.y << ","
                         << odom_transform.transform.rotation.z << ","
                         << odom_transform.transform.rotation.w << ")" << std::endl;

        odom_transform.header.frame_id = base_frame;
        odom_transform.child_frame_id = child_frame;
        odom_transform.header.stamp = ros::Time::now(); // Add time to the odometry data

        odom_broadcaster.sendTransform(odom_transform); // Broadcast

        ros::spinOnce();
        r.sleep();
    }
    return 0;
}
