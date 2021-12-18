#ifndef DETECTARUCO_H
#define DETECTARUCO_H

#define EXPLICIT_CONSTRUCTOR 1
#define EXPLICIT_DESTRUCTOR 1

// Image conversion from ROS to CV format libraries
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// Time 
#include <time.h>

// Custom msg
#include <operator_intent_msgs/point2d.h>
#include <operator_intent_msgs/corner_array.h>
#include <operator_intent_msgs/marker_locations.h>

// Aruco relevant libraries
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp> // <-- was not needed, as it's already included in line 13
#include <iostream>


class DetectAruco
{
private:
    const std::string OPENCV_WINDOW = "Aruco Marker Detection";
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    ros::Publisher markers_loc_pub;
    std::string m_pub_topic, m_sub_rgb_image_topic;

    // Initialize vectors for the ID's of the markers and the marker corners
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f> > markerCorners;

    // The below are commented out, as they are only for the rejected candidates for tags
    // std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
    // cv::Ptr<cv::aruco::DetectorParameters> parameters;

    // Define the dictionary to be detected
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);

public:
#if EXPLICIT_CONSTRUCTOR
    DetectAruco(std::string sub_rgb_image_topic, std::string pub_topic);
#endif
#if EXPLICIT_DESTRUCTOR
    ~DetectAruco();
#endif
};

#endif
