// Image conversion from ROS to CV format libraries
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// Custom msg
#include <operator_intent_msgs/point_2d.h>
#include <operator_intent_msgs/corner_array.h>
#include <operator_intent_msgs/marker_locations.h>

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
    ros::Publisher orthogonal_markers_loc_pub_;
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
    DetectAruco(std::string sub_rgb_image_topic, std::string pub_topic);
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
    /*
    cv::aruco::drawDetectedMarkers(cv_ptr->image, marker_corners, marker_ids);
    */
    // Checking if the marker_corners array is empty, if it is, we skip it (keep in mind the
    // dimensions of the marker_corners array is Nx4, where N is the number of tags detected)

    std::vector<std::vector<cv::Point2f> > orthogonal_marker_corners;
    // The code below will attempt to draw a bounding box containing the tag, if any tags were detected successfully
    if (marker_corners.size() > 0)
    {
        //std::vector<std::vector<cv::Point2f> > orthogonal_marker_corners = marker_corners;
        orthogonal_marker_corners = marker_corners;
        // Loop through all of the detected tags
        float min_x, min_y, max_x, max_y;
        for (unsigned long int i = 0; i < orthogonal_marker_corners.size(); i++)
        {
            // Initialize the minimum and maximum values of the corners to the marker's first corner values
            min_x = marker_corners[i][0].x;
            min_y = marker_corners[i][0].y;
            max_x = marker_corners[i][0].x;
            max_y = marker_corners[i][0].y;

            // Check for the minimum and maximum values of x and y of all the corners, in order to draw an orthogonal bounding box.

            /* The reason for the orthogonal bounding box instead of the one provided by the drawDetectedMarkers method
                * offered in the aruco library is for the sole purpose of iterating through the pixels of the image
                * for the depth information. The drawDetectedMarkers will provide boundaries for the
                * markers, however it might not be orthogonal and that introduces problems for the logic. */
            for (unsigned long int j = 0; j < 4; j++)
            {
                if (marker_corners[i][j].x < min_x) min_x = marker_corners[i][j].x;
                if (marker_corners[i][j].x > max_x) max_x = marker_corners[i][j].x;
                if (marker_corners[i][j].y < min_y) min_y = marker_corners[i][j].y;
                if (marker_corners[i][j].y > max_y) max_y = marker_corners[i][j].y;
            }

            // Quick and dirty way to assign corners to orthogonal_marker_corners

            for (unsigned long int j = 0; j < 4; j++)
            {
                switch (j)
                {
                case 0:
                    orthogonal_marker_corners[i][j].x = min_x;
                    orthogonal_marker_corners[i][j].y = min_y;
                    break;
                case 1:
                    orthogonal_marker_corners[i][j].x = max_x;
                    orthogonal_marker_corners[i][j].y = min_y;
                    break;
                case 2:
                    orthogonal_marker_corners[i][j].x = max_x;
                    orthogonal_marker_corners[i][j].y = max_y;
                    break;
                case 3:
                    orthogonal_marker_corners[i][j].x = min_x;
                    orthogonal_marker_corners[i][j].y = max_y;
                    break;
                }
            }

            // Line Thickness for drawing the borders
            int thickness = 1;

            // Configure the color for the lines
            cv::Scalar color = cv::Scalar(0, 255, 0);

            // Loop through the orthogonal_marker_corners and draw the bounding boxes
            for (unsigned int j = 0; j < 3; j++)
            {
                cv::line(cv_ptr->image, orthogonal_marker_corners[i][j], orthogonal_marker_corners[i][j+1], color, thickness, cv::LINE_8);
            }
            cv::line(cv_ptr->image, orthogonal_marker_corners[i][3], orthogonal_marker_corners[i][0], color, thickness, cv::LINE_8);
        }
    }

    // Publish the orthognal marker corners to the topic "aruco/orthogonal_markers_loc"
    // As well as the original marker corners to the topic "aruco/markers_loc"
    if (orthogonal_marker_corners.size() > 0)
    {
        operator_intent_msgs::marker_locations orthogonal_marker_locations;
        operator_intent_msgs::marker_locations marker_locations;
        for (unsigned long int i = 0; i < orthogonal_marker_corners.size(); i++){
            operator_intent_msgs::corner_array orthogonal_corner_array;
            operator_intent_msgs::corner_array corner_array;
            orthogonal_corner_array.markerId = marker_ids[i];
            corner_array.markerId = marker_ids[i];
            for (unsigned long int j = 0; j < 4; j++)
            {
                operator_intent_msgs::point_2d orthogonal_point_2d;
                operator_intent_msgs::point_2d point_2d;
                orthogonal_point_2d.x = orthogonal_marker_corners[i][j].x;
                orthogonal_point_2d.y = orthogonal_marker_corners[i][j].y;
                orthogonal_corner_array.corner_points[j] = orthogonal_point_2d;
                point_2d.x = marker_corners[i][j].x;
                point_2d.y = marker_corners[i][j].y;
                corner_array.corner_points[j] = point_2d;
            }
            orthogonal_marker_locations.header.stamp = ros::Time::now();
            marker_locations.header.stamp = ros::Time::now();
            orthogonal_marker_locations.n_markers, marker_locations.n_markers = orthogonal_marker_corners.size();
            orthogonal_marker_locations.markers.push_back(orthogonal_corner_array);
            marker_locations.markers.push_back(corner_array);

        }
        orthogonal_markers_loc_pub_.publish(orthogonal_marker_locations);
        markers_loc_pub_.publish(marker_locations);
    }

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
    markers_loc_pub_ = nh_.advertise<operator_intent_msgs::marker_locations>("aruco/markers_loc", 1);
    orthogonal_markers_loc_pub_ = nh_.advertise<operator_intent_msgs::marker_locations>("aruco/orthogonal_markers_loc", 1);

    cv::namedWindow(OPENCV_WINDOW);
}

DetectAruco::DetectAruco(std::string sub_rgb_image_topic, std::string pub_topic)
    :it_(nh_)
{
    sub_rgb_image_topic_ = sub_rgb_image_topic;
    pub_topic_ = pub_topic;
    // Subscribe to input video feed and publish output video feed
    image_sub_ = it_.subscribe(sub_rgb_image_topic_, 1,
      &DetectAruco::imageCallback, this);
    image_pub_ = it_.advertise(pub_topic_, 1);
    markers_loc_pub_ = nh_.advertise<operator_intent_msgs::marker_locations>("aruco/markers_loc", 1);
    orthogonal_markers_loc_pub_ = nh_.advertise<operator_intent_msgs::marker_locations>("aruco/orthogonal_markers_loc", 1);

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