#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <operator_intent_msgs/marker_collection.h>
#include "node_template.cpp"


class SubscribeAndPublish
{

    public:
    SubscribeAndPublish(ros::NodeHandle nh, ros::NodeHandle pnh);
    ~SubscribeAndPublish();

    

    private:

    //ros::Publisher odom_pub;

    message_filters::Subscriber<sensor_msgs::Image> image_sub;

    message_filters::Subscriber<sensor_msgs::Image> image_sub2;

    void callback(const sensor_msgs::ImageConstPtr &image, const sensor_msgs::ImageConstPtr &image2);
    
};

SubscribeAndPublish::SubscribeAndPublish(ros::NodeHandle nh, ros::NodeHandle pnh){      

    image_sub.subscribe(nh, "/camera/rgb/image_raw", 1);
    image_sub2.subscribe(nh, "/camera/depth/image_raw", 1);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;

    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image_sub, image_sub2);
    sync.registerCallback(boost::bind(&SubscribeAndPublish::callback, this, _1, _2));
    ROS_INFO("3x");
    //odom_pub = n.advertise<nav_msgs::Odometry>("RGBDodom", 50);
    ros::spin();
}

SubscribeAndPublish::~SubscribeAndPublish() {
    std::cout << "End of the line!" << std::endl;
}

void SubscribeAndPublish::callback(const sensor_msgs::ImageConstPtr &image, const sensor_msgs::ImageConstPtr &image2) {
    std::cout << "Hello from the callback function!\nIf you're seeing this, you're a smart guy!" << std::endl;
    //std::cout << (double)image2->data[0] << std::endl;
    std::cout << (double)sizeof(image2->data) / sizeof(image2->data[0]) << std::endl;
    std::cout << image2->height << std::endl;
    std::cout << image2->width << std::endl;
}

int main(int argc, char **argv) {
    NodeMain<SubscribeAndPublish>(argc, argv, "SubscribeAndPublishNode");
}

/*
int main(int argc, char **argv) {
    ros::init(argc, argv, "testnode");
    ros::NodeHandle nh;
    ros::NodeHandle pnh;
    SubscribeAndPublish a(nh, pnh);
}
*/