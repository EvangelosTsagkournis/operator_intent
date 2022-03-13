#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <operator_intent_msgs/point2d.h>
#include <operator_intent_msgs/corner_array.h>
#include <operator_intent_msgs/marker_locations.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <iostream>


static const std::string OPENCV_WINDOW = "Image Window";

class CalculateArucoDistance
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  //message_filters::Subscriber<operator_intent_msgs::marker_locations> marker_loc_sub_;

public:
  CalculateArucoDistance();
  ~CalculateArucoDistance();

  void callBack(const operator_intent_msgs::marker_locationsConstPtr &markers, const sensor_msgs::ImageConstPtr &image);
  void callBack(const sensor_msgs::ImageConstPtr& msg);
};