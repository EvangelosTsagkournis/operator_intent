/*This node outputs the depth image from the camera/depth/image_raw topic
 * in an opencv window. It was made for debugging purposes and can
 * be safely excluded from compilation.
 */

//Image conversion from ROS to CV format libraries
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

//#include <iostream>

static const std::string OPENCV_WINDOW = "Image window";

class Depth_Image
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  Depth_Image()
    : it_(nh_)
  {
    // Subscribe to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/depth/image_raw", 1,
      &Depth_Image::imageCallback, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW);
  }
/*
  ~Depth_Image()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }
*/

  void imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1); // sensor_msgs::image_encodings::BGR8
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Normalize the image from [0, 255] to [0, 1]

    //cv::Mat norm_image;
    //cv_ptr->image.convertTo(norm_image, CV_32F, 1.0/255, 0);
    cv::normalize(cv_ptr->image, cv_ptr->image, 0, 1, cv::NORM_MINMAX, CV_32F);

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    //cv::imshow(OPENCV_WINDOW, norm_image);
    cv::waitKey(3);

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  Depth_Image di;
  ros::spin();
  return 0;
}
