#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <operator_intent_msgs/point_2d.h>
#include <operator_intent_msgs/corner_array.h>
#include <operator_intent_msgs/marker_locations.h>
#include <operator_intent_msgs/pixel_with_distance.h>
#include <operator_intent_msgs/pixel_array.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <iostream>

#include "node_template.cpp"

class FindObjectPosition {
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    
};