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
#include <operator_intent_msgs/pixel_coordinates_with_distance.h>
#include <operator_intent_msgs/pixel_coordinates_with_distance_collection.h>

#include <cmath>
#include <iostream>

// Node template
#include "node_template.cpp"

// static const std::string OPENCV_WINDOW = "Image Window";

// Define Infinite (Using INT_MAX caused overflow problems)
#define INF 10000

// Found by searching for the kinect_camera.urdf.xacro file in husky/husky_description/urdf/accessories
#define KINECT_CAMERA_HORIZONTAL_FOV_DEG 70

#define SCREEN_WIDTH 640
#define SCREEN_HEIGHT 480

class CalculateObjectDistanceAndAngle
{
private:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  image_transport::SubscriberFilter depth_image_sub_;
  message_filters::Subscriber<operator_intent_msgs::marker_collection> marker_loc_sub_;
  ros::Publisher pixel_coordinates_with_distance_collection_pub_;

  typedef union U_FloatParse
  {
    float float_data;
    unsigned char byte_data[4];
  } U_FloatConvert;

  int readDepthData(cv::Point2i, sensor_msgs::ImageConstPtr depth_image);
  double findAngleInRadians(cv::Point2i);

  bool intersection(cv::Point2i o1, cv::Point2i p1, cv::Point2i o2, cv::Point2i p2, cv::Point2i &r);

public:
  CalculateObjectDistanceAndAngle();
  CalculateObjectDistanceAndAngle(ros::NodeHandle, ros::NodeHandle);
  ~CalculateObjectDistanceAndAngle();

  void callBack(
      const operator_intent_msgs::marker_collectionConstPtr &marker_collection,
      const sensor_msgs::ImageConstPtr &image);
};

CalculateObjectDistanceAndAngle::CalculateObjectDistanceAndAngle(ros::NodeHandle nh, ros::NodeHandle pnh) : it_(nh_),
                                                                                                            depth_image_sub_(it_, "camera/depth/image_raw", 1)
{
  // Subscribe to input video feed and publish output video feed
  marker_loc_sub_.subscribe(nh_, "/aruco/markers_loc", 1);
  pixel_coordinates_with_distance_collection_pub_ = nh_.advertise<operator_intent_msgs::pixel_coordinates_with_distance_collection>("/aruco/pixel_coordinates_with_distance_collection", 1);
  // depth_image_sub_.subscribe(nh_, "/camera/depth/image_raw", 1);

  typedef message_filters::sync_policies::ApproximateTime<operator_intent_msgs::marker_collection, sensor_msgs::Image> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), marker_loc_sub_, depth_image_sub_);
  sync.registerCallback(boost::bind(&CalculateObjectDistanceAndAngle::callBack, this, _1, _2));
  ros::spin();
}

CalculateObjectDistanceAndAngle::~CalculateObjectDistanceAndAngle()
{
}

int CalculateObjectDistanceAndAngle::readDepthData(cv::Point2i intersection_point, sensor_msgs::ImageConstPtr depth_image)
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

double CalculateObjectDistanceAndAngle::findAngleInRadians(cv::Point2i intersection_point)
{
  cv::Point2i middle_pixel(SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2);
  double degrees_per_pixel = (double)KINECT_CAMERA_HORIZONTAL_FOV_DEG / (double)SCREEN_WIDTH;
  int difference_in_x = middle_pixel.x - intersection_point.x;
  double angleInDegrees = degrees_per_pixel * (middle_pixel.x - intersection_point.x);
  return angleInDegrees * M_PI / 180;
}

// Finds the intersection of two lines, or returns false.
// The lines are defined by (o1, p1) and (o2, p2).
bool CalculateObjectDistanceAndAngle::intersection(cv::Point2i o1, cv::Point2i p1, cv::Point2i o2, cv::Point2i p2, cv::Point2i &r)
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

void CalculateObjectDistanceAndAngle::callBack(
    const operator_intent_msgs::marker_collectionConstPtr &marker_collection,
    const sensor_msgs::ImageConstPtr &image)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::TYPE_32FC1);
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  operator_intent_msgs::pixel_coordinates_with_distance_collection pixel_coordinates_with_distance_collection;
  // For each marker [0...n]:
  for (unsigned long int i = 0; i < marker_collection->markers.size(); i++)
  {
    operator_intent_msgs::pixel_coordinates_with_distance pixel_coordinates_with_distance;
    cv::Point2i marker_points[4];
    cv::Point2i intersection_point;
    for (int j = 0; j < (sizeof(marker_points) / sizeof(marker_points[0])); j++)
    {
      marker_points[j] = cv::Point2i(
          marker_collection->markers[i].corner_points[j].x,
          marker_collection->markers[i].corner_points[j].y);
    }
    if (intersection(marker_points[0], marker_points[2], marker_points[1], marker_points[3], intersection_point))
    {
      pixel_coordinates_with_distance.distance = readDepthData(intersection_point, image);
      pixel_coordinates_with_distance.angle_radians = findAngleInRadians(intersection_point);
      /*
      std::cout
        << "The intersection point for marker id#" << marker_collection->markers[i].markerId
        << " has coordinates: x = " << intersection_point.x << ", y = "
        << intersection_point.y << std::endl;
      std::cout
        << "The depth for the marker #"
        << i <<  " and id: " << marker_collection->markers[i].markerId << " is: "
        << (int)pixel_coordinates_with_distance.distance
        << " mm" <<  " and the angle from the camera POV is: "
        << pixel_coordinates_with_distance.angle_radians << std::endl;
      */
      pixel_coordinates_with_distance.pixel_x = intersection_point.x;
      pixel_coordinates_with_distance.pixel_y = intersection_point.y;
      pixel_coordinates_with_distance_collection.pixels.push_back(pixel_coordinates_with_distance);
    }
  }
  pixel_coordinates_with_distance_collection.header.stamp = ros::Time::now();
  pixel_coordinates_with_distance_collection.camera_height = image->height;
  pixel_coordinates_with_distance_collection.camera_width = image->width;

  pixel_coordinates_with_distance_collection_pub_.publish(pixel_coordinates_with_distance_collection);
}

int main(int argc, char **argv)
{
  NodeMain<CalculateObjectDistanceAndAngle>(argc, argv, "CalculateObjectDistanceAndAngleNode");
}
