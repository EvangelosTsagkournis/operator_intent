// Standard C++ libraries

#include <cmath>
#include <iostream>

// Image conversion from ROS to CV format libraries
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// Custom msgs
#include <operator_intent_msgs/point_2dc.h>
#include <operator_intent_msgs/marker.h>
#include <operator_intent_msgs/marker_collection.h>
#include <operator_intent_msgs/marker_coordinates_with_distance.h>
#include <operator_intent_msgs/marker_coordinates_with_distance_collection.h>

// Aruco relevant libraries
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp> // <-- was not needed, as it's already included in line 14

// Node template
#include "node_template.cpp"

// Define Infinite (Using INT_MAX caused overflow problems)
#define INF 10000

// Found by searching for the kinect_camera.urdf.xacro file in husky/husky_description/urdf/accessories
#define KINECT_CAMERA_HORIZONTAL_FOV_DEG 70

#define SCREEN_WIDTH 640
#define SCREEN_HEIGHT 480

#define RGB_TOPIC "camera/rgb/image_raw"
#define DEPTH_TOPIC "camera/depth/image_raw"

class DetectAruco
{
private:
    const std::string OPENCV_WINDOW = "Aruco Marker Detection";
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::SubscriberFilter rgb_image_sub_;
    image_transport::SubscriberFilter depth_image_sub_;
    ros::Publisher marker_coordinates_with_distance_collection_pub_;

    typedef union U_FloatParse
    {
        float float_data;
        unsigned char byte_data[4];
    } U_FloatConvert;

    // Initialize vectors for the ID's of the markers and the marker corners
    std::vector<int> marker_ids;
    std::vector<std::vector<cv::Point2f>> marker_corners;

    // Define the dictionary to be detected
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    void callBack(const sensor_msgs::ImageConstPtr &rgb_image, const sensor_msgs::ImageConstPtr &depth_image);

    bool onSegment(cv::Point2i, cv::Point2i, cv::Point2i);
    int orientation(cv::Point2i, cv::Point2i, cv::Point2i);
    bool doIntersect(cv::Point2i, cv::Point2i, cv::Point2i, cv::Point2i);
    bool isInside(cv::Point2i[], cv::Point2i);

    int readDepthData(cv::Point2i, sensor_msgs::ImageConstPtr);
    double findAngleInRadians(cv::Point2i);

    bool intersection(cv::Point2i o1, cv::Point2i p1, cv::Point2i o2, cv::Point2i p2, cv::Point2i &r);

public:
    DetectAruco();
    DetectAruco(ros::NodeHandle, ros::NodeHandle);
    ~DetectAruco();
};

DetectAruco::DetectAruco(ros::NodeHandle nh, ros::NodeHandle pnh)
    : it_(nh_),
      rgb_image_sub_(it_, RGB_TOPIC, 1),
      depth_image_sub_(it_, DEPTH_TOPIC, 1)
{
    marker_coordinates_with_distance_collection_pub_ = nh_.advertise<operator_intent_msgs::marker_coordinates_with_distance_collection>("/aruco/marker_coordinates_with_distance_collection", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), rgb_image_sub_, depth_image_sub_);
    sync.registerCallback(boost::bind(&DetectAruco::callBack, this, _1, _2));

    ros::spin();
    cv::namedWindow(OPENCV_WINDOW);
}

DetectAruco::~DetectAruco()
{
    cv::destroyWindow(OPENCV_WINDOW);
}

int DetectAruco::readDepthData(cv::Point2i intersection_point, sensor_msgs::ImageConstPtr depth_image)
{
    // If position is invalid
    if ((intersection_point.y >= depth_image->height) || (intersection_point.x >= depth_image->width))
        return -1;
    int index = (intersection_point.y * depth_image->step) + (intersection_point.x * (depth_image->step / depth_image->width));
    // If data is 4 byte floats (rectified depth image)
    if ((depth_image->step / depth_image->width) == 4)
    {
        U_FloatConvert depth_data;
        int i, endian_check = 1;
        // If big endian
        if ((depth_image->is_bigendian && (*(char *)&endian_check != 1)) || // Both big endian
            ((!depth_image->is_bigendian) && (*(char *)&endian_check == 1)))
        { // Both lil endian
            for (i = 0; i < 4; i++)
                depth_data.byte_data[i] = depth_image->data[index + i];
            // Make sure data is valid (check if NaN)
            if (depth_data.float_data == depth_data.float_data)
                return int(depth_data.float_data * 1000);
            return -1; // If depth data invalid
        }
        // else, one little endian, one big endian
        for (i = 0; i < 4; i++)
            depth_data.byte_data[i] = depth_image->data[3 + index - i];
        // Make sure data is valid (check if NaN)
        if (depth_data.float_data == depth_data.float_data)
            return int(depth_data.float_data * 1000);
        return -1; // If depth data invalid
    }
    // Otherwise, data is 2 byte integers (raw depth image)
    int temp_val;
    // If big endian
    if (depth_image->is_bigendian)
        temp_val = (depth_image->data[index] << 8) + depth_image->data[index + 1];
    // If little endian
    else
        temp_val = depth_image->data[index] + (depth_image->data[index + 1] << 8);
    // Make sure data is valid (check if NaN)
    if (temp_val == temp_val)
        return temp_val;
    return -1; // If depth data invalid
}

double DetectAruco::findAngleInRadians(cv::Point2i intersection_point)
{
    cv::Point2i middle_pixel(SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2);
    double degrees_per_pixel = (double)KINECT_CAMERA_HORIZONTAL_FOV_DEG / (double)SCREEN_WIDTH;
    int difference_in_x = middle_pixel.x - intersection_point.x;
    double angleInDegrees = degrees_per_pixel * (middle_pixel.x - intersection_point.x);
    return angleInDegrees * M_PI / 180;
}

// Finds the intersection of two lines, or returns false.
// The lines are defined by (o1, p1) and (o2, p2).
bool DetectAruco::intersection(cv::Point2i o1, cv::Point2i p1, cv::Point2i o2, cv::Point2i p2, cv::Point2i &r)
{
    cv::Point2i x = o2 - o1;
    cv::Point2i d1 = p1 - o1;
    cv::Point2i d2 = p2 - o2;

    float cross = d1.x * d2.y - d1.y * d2.x;
    if (abs(cross) < /*EPS*/ 1e-8)
        return false;

    double t1 = (x.x * d2.y - x.y * d2.x) / cross;
    r = o1 + d1 * t1;
    return true;
}

void DetectAruco::callBack(const sensor_msgs::ImageConstPtr &rgb_image, const sensor_msgs::ImageConstPtr &depth_image)
{
    cv_bridge::CvImagePtr cv_ptr_rgb;
    cv_bridge::CvImagePtr cv_ptr_depth;
    try
    {
        cv_ptr_rgb = cv_bridge::toCvCopy(rgb_image, sensor_msgs::image_encodings::BGR8);
        cv_ptr_depth = cv_bridge::toCvCopy(depth_image, sensor_msgs::image_encodings::TYPE_32FC1);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // Detect tags in the image, using the provided detectMarkers method
    cv::aruco::detectMarkers(cv_ptr_rgb->image, dictionary, marker_corners, marker_ids);
    // Draw the bounding box around the detected tags using the provided drawDetectedMarkers method
    // Commented out for debugging
    cv::aruco::drawDetectedMarkers(cv_ptr_rgb->image, marker_corners, marker_ids);
    // Checking if the marker_corners array is empty, if it is, we skip it (keep in mind the
    // dimensions of the marker_corners array is Nx4, where N is the number of tags detected)

    operator_intent_msgs::marker_coordinates_with_distance_collection marker_coordinates_with_distance_collection;
    // Looping through all of the detected markers using their ID's
    for (unsigned long int i = 0; i < marker_ids.size(); i++)
    {
        operator_intent_msgs::marker_coordinates_with_distance marker_coordinates_with_distance;
        cv::Point2i marker_points[4];
        cv::Point2i intersection_point;
        for (unsigned long int j = 0; j < 4; j++)
        {
            marker_points[j] = cv::Point2i(marker_corners[i][j].x, marker_corners[i][j].y);
        }
        if (intersection(marker_points[0], marker_points[2], marker_points[1], marker_points[3], intersection_point))
        {
            marker_coordinates_with_distance.distance = readDepthData(intersection_point, depth_image);
            marker_coordinates_with_distance.angle_radians = findAngleInRadians(intersection_point);
            marker_coordinates_with_distance.marker_pixel_x = intersection_point.x;
            marker_coordinates_with_distance.marker_pixel_y = intersection_point.y;
            marker_coordinates_with_distance_collection.markers.push_back(marker_coordinates_with_distance);
        }
    }
    marker_coordinates_with_distance_collection.camera_height = rgb_image->height;
    marker_coordinates_with_distance_collection.camera_width = rgb_image->width;
    marker_coordinates_with_distance_collection.header.stamp = ros::Time::now();
    marker_coordinates_with_distance_collection_pub_.publish(marker_coordinates_with_distance_collection);

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr_rgb->image);
    cv::waitKey(3);
}

int main(int argc, char **argv)
{
    NodeMain<DetectAruco>(argc, argv, "DetectArucoNode");
}