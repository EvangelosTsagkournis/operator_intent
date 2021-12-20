// Image conversion from ROS to CV format libraries
/*
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// Aruco relevant libraries
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>
//#include <opencv2/highgui.hpp> // <-- was not needed, as it's already included in line 13
#include <iostream>
*/
#include <detect_aruco.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  std::string sub_topic = "/camera/rgb/image_raw";
  std::string pub_topic = "/image_converter/output_video";
  DetectAruco da(sub_topic, pub_topic);
  ros::spin();
  return 0;
}
