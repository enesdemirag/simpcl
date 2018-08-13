// Import dependencies
#include <ros/ros.h>

// Definitions
ros::Publisher pub;

std::string subscribed_topic;
std::string published_topic;

altitude0, altitude1 = 0;
double threshold;

// callback function
void cloud_cb(const double msg)
{
    altitude1 = altitude0;
    altitude0 = msg;
}

// main function
int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv, "altitude_listener");
    ros::NodeHandle n;
    ros::Rate r(20);
    // Load parameters from launch file
    ros::NodeHandle nh_private("~");
    nh_private.param<std::string>("subscribed_topic", subscribed_topic, "/laserscan");
    nh_private.param<std::string>("published_topic", published_topic, "altitude_tracker");
    nh_private.param<double>("threshold", threshold, 0.5);

    // Create Subscriber and listen subscribed_topic
    ros::Subscriber sub = n.subscribe<double>(subscribed_topic, 1, cloud_cb);
    // Create Publisher
    pub = n.advertise<double>(published_topic, 1);

    while(true)
    {
        if(abs(altitude1 - altitude0) < threshold))
        {
            pub.publish(altitude1); // There is no obstacle between drone and ground
        }
        else
        {
            while(true)
            {
                if(abs(altitude1 - altitude0 < threshold)
                {
                    pub.publish(0); // Obstacle found. Do not remove points under obstacle
                }
                else
                {
                    break;
                }
            }
        }
        r.sleep();
        ros::spin();
    }
    return 0;
}
