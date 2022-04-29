// Image conversion from ROS to CV format libraries
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// Custom msg
#include <operator_intent_msgs/point_2dc.h>
#include <operator_intent_msgs/marker.h>
#include <operator_intent_msgs/marker_collection.h>

// Aruco relevant libraries
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp> // <-- was not needed, as it's already included in line 13
#include <iostream>

#include "node_template.cpp"


class DetectAruco
{
private:
    const std::string OPENCV_WINDOW = "Aruco Marker Detection";
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    ros::Publisher markers_loc_pub_;
    std::string pub_topic_, sub_rgb_image_topic_;

    // Initialize vectors for the ID's of the markers and the marker corners
    std::vector<int> marker_ids;
    std::vector<std::vector<cv::Point2f> > marker_corners;

    // The below are commented out, as they are only for the rejected candidates for tags
    // std::vector<std::vector<cv::Point2f>> marker_corners, rejectedCandidates;
    // cv::Ptr<cv::aruco::DetectorParameters> parameters;

    // Define the dictionary to be detected
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);

public:
    DetectAruco(ros::NodeHandle, ros::NodeHandle);
    ~DetectAruco();
};


void DetectAruco::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // Detect tags in the image, using the provided detectMarkers method
    cv::aruco::detectMarkers(cv_ptr->image, dictionary, marker_corners, marker_ids);
    // Draw the bounding box around the detected tags using the provided drawDetectedMarkers method
    // Commented out for debugging
    cv::aruco::drawDetectedMarkers(cv_ptr->image, marker_corners, marker_ids);
    // Checking if the marker_corners array is empty, if it is, we skip it (keep in mind the
    // dimensions of the marker_corners array is Nx4, where N is the number of tags detected)

    // Publish the marker corners to the topic "aruco/markers_loc"
    operator_intent_msgs::marker_collection marker_collection;
    for (unsigned long int i = 0; i < marker_corners.size(); i++){
        operator_intent_msgs::marker marker;
        marker.markerId = marker_ids[i];
        for (unsigned long int j = 0; j < 4; j++)
        {
            operator_intent_msgs::point_2dc point_2dc;
            point_2dc.x = marker_corners[i][j].x;
            point_2dc.y = marker_corners[i][j].y;
            marker.corner_points[j] = point_2dc;
        }
        // marker_collection.header.stamp = ros::Time::now();
        marker_collection.markers.push_back(marker);
    }
    marker_collection.header.stamp = ros::Time::now();
    markers_loc_pub_.publish(marker_collection);

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);

    // Output modified video stream
    //image_pub_.publish(cv_ptr->toImageMsg());

}

DetectAruco::DetectAruco(ros::NodeHandle nh, ros::NodeHandle pnh)
    :it_(nh_)
{
    sub_rgb_image_topic_ = "/camera/rgb/image_raw";
    pub_topic_ = "/image_converter/output_video";
    // Subscribe to input video feed and publish output video feed
    image_sub_ = it_.subscribe(sub_rgb_image_topic_, 1,
      &DetectAruco::imageCallback, this);
    image_pub_ = it_.advertise(pub_topic_, 1);
    markers_loc_pub_ = nh_.advertise<operator_intent_msgs::marker_collection>("aruco/markers_loc", 1);

    cv::namedWindow(OPENCV_WINDOW);
}


DetectAruco::~DetectAruco()
{
    cv::destroyWindow(OPENCV_WINDOW);
}


// int main(int argc, char** argv)
// {
//   ros::init(argc, argv, "image_converter");
//   std::string sub_topic = "/camera/rgb/image_raw";
//   std::string pub_topic = "/image_converter/output_video";
//   DetectAruco da(sub_topic, pub_topic);
//   ros::spin();
//   return 0;
// }

int main(int argc, char** argv)
{
    NodeMain<DetectAruco>(argc, argv, "DetectArucoNode");
}