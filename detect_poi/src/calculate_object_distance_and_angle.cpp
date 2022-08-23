#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

// Custom msgs
#include <operator_intent_msgs/point_2dc.h>
#include <operator_intent_msgs/marker.h>
#include <operator_intent_msgs/marker_collection.h>
#include <operator_intent_msgs/marker_coordinates_with_distance.h>
#include <operator_intent_msgs/marker_coordinates_with_distance_collection.h>

#include <cmath>
#include <iostream>

// Node template
#include "node_template.cpp"

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
  ros::NodeHandle pnh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  image_transport::SubscriberFilter depth_image_sub_;
  message_filters::Subscriber<operator_intent_msgs::marker_collection> marker_loc_sub_;
  message_filters::Subscriber<nav_msgs::Odometry> odometry_sub_;
  ros::Publisher marker_coordinates_with_distance_collection_pub_;

  typedef union U_FloatParse
  {
    float float_data;
    unsigned char byte_data[4];
  } U_FloatConvert;

  int readDepthData(cv::Point2i, sensor_msgs::ImageConstPtr depth_image);
  double findAngleInRadiansFromCameraPointOfReference(cv::Point2i);
  double findAngleInRadiansFromQuaternion(geometry_msgs::Quaternion &quaternion);

  bool findPositionOfMarker(
      double &distance,
      double &angle_radians,
      double &robot_angle_radians,
      double robot_position_x,
      double robot_position_y,
      double &marker_position_x,
      double &marker_position_y);

  bool intersection(cv::Point2i o1, cv::Point2i p1, cv::Point2i o2, cv::Point2i p2, cv::Point2i &r);

public:
  CalculateObjectDistanceAndAngle();
  CalculateObjectDistanceAndAngle(ros::NodeHandle, ros::NodeHandle);
  ~CalculateObjectDistanceAndAngle();

  void callBack(
      const operator_intent_msgs::marker_collectionConstPtr &marker_collection,
      const sensor_msgs::ImageConstPtr &image,
      const nav_msgs::OdometryConstPtr &odometry);
};

CalculateObjectDistanceAndAngle::CalculateObjectDistanceAndAngle(ros::NodeHandle nh, ros::NodeHandle pnh)
    : nh_(nh), pnh_(pnh_), it_(nh_), depth_image_sub_(it_, "camera/depth/image_raw", 1)
{
  // Subscribe to input video feed and publish output video feed
  marker_loc_sub_.subscribe(nh_, "/aruco/markers_loc", 1);
  odometry_sub_.subscribe(nh_, "husky_base_ground_truth", 1);
  marker_coordinates_with_distance_collection_pub_ = nh_.advertise<operator_intent_msgs::marker_coordinates_with_distance_collection>("/aruco/marker_coordinates_with_distance_collection", 1);

  typedef message_filters::sync_policies::ApproximateTime<operator_intent_msgs::marker_collection, sensor_msgs::Image, nav_msgs::Odometry> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), marker_loc_sub_, depth_image_sub_, odometry_sub_);
  sync.registerCallback(boost::bind(&CalculateObjectDistanceAndAngle::callBack, this, _1, _2, _3));
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
    if ((depth_image->is_bigendian && (*(char *)&endian_check != 1)) ||  // Both big endian
        ((!depth_image->is_bigendian) && (*(char *)&endian_check == 1))) // Both lil endian
    {
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

double CalculateObjectDistanceAndAngle::findAngleInRadiansFromCameraPointOfReference(cv::Point2i intersection_point)
{
  cv::Point2i middle_pixel(SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2);
  double degrees_per_pixel = (double)KINECT_CAMERA_HORIZONTAL_FOV_DEG / (double)SCREEN_WIDTH;
  int difference_in_x = middle_pixel.x - intersection_point.x;
  double angleInRadians = (degrees_per_pixel * (middle_pixel.x - intersection_point.x)) * M_PI / 180;
  return angleInRadians;
}

bool CalculateObjectDistanceAndAngle::findPositionOfMarker(
    double &distance_mm,
    double &angle_radians,
    double &robot_angle_radians,
    double robot_position_x,
    double robot_position_y,
    double &marker_position_x,
    double &marker_position_y)
{
  if (distance_mm != -1.0)
  {
    double consolidated_angle_radians = robot_angle_radians + angle_radians;
    marker_position_x = robot_position_x + distance_mm / 1000 * cos(consolidated_angle_radians);
    marker_position_y = robot_position_y + distance_mm / 1000 * sin(consolidated_angle_radians);
    return true;
  }
  return false;
}

double CalculateObjectDistanceAndAngle::findAngleInRadiansFromQuaternion(geometry_msgs::Quaternion &quaternion)
{
  tf2::Quaternion q(
      quaternion.x,
      quaternion.y,
      quaternion.z,
      quaternion.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  return yaw;
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
    const sensor_msgs::ImageConstPtr &image,
    const nav_msgs::OdometryConstPtr &odometry)
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

  // Set robot angle
  geometry_msgs::Quaternion temp_quaternion = odometry->pose.pose.orientation;
  double robot_angle = findAngleInRadiansFromQuaternion(temp_quaternion);

  operator_intent_msgs::marker_coordinates_with_distance_collection marker_coordinates_with_distance_collection;
  // For each marker [0...n]:
  for (unsigned long int i = 0; i < marker_collection->markers.size(); i++)
  {
    operator_intent_msgs::marker_coordinates_with_distance marker_coordinates_with_distance;
    cv::Point2i marker_points[4];
    cv::Point2i intersection_point;
    for (int j = 0; j < (sizeof(marker_points) / sizeof(marker_points[0])); j++)
    {
      marker_points[j] = cv::Point2i(
          marker_collection->markers[i].corner_points[j].x,
          marker_collection->markers[i].corner_points[j].y);
    }
    // If the four corners of the marker intersect:
    if (intersection(marker_points[0], marker_points[2], marker_points[1], marker_points[3], intersection_point))
    {
      // Assign the marker's ID
      marker_coordinates_with_distance.marker_id = marker_collection->markers[i].marker_id;
      // Write the depth data
      marker_coordinates_with_distance.distance_mm = readDepthData(intersection_point, image);
      // Write the angle in radians
      marker_coordinates_with_distance.angle_radians = findAngleInRadiansFromCameraPointOfReference(intersection_point);

      double marker_x, marker_y;
      // Find marker position in world
      if (findPositionOfMarker(
              marker_coordinates_with_distance.distance_mm,
              marker_coordinates_with_distance.angle_radians,
              robot_angle,
              odometry->pose.pose.position.x,
              odometry->pose.pose.position.y,
              marker_x,
              marker_y))
      {
        marker_coordinates_with_distance.marker_world_x = marker_x;
        marker_coordinates_with_distance.marker_world_y = marker_y;
      }
      // Assign the center of the marker
      marker_coordinates_with_distance.marker_pixel_x = intersection_point.x;
      marker_coordinates_with_distance.marker_pixel_y = intersection_point.y;

      marker_coordinates_with_distance_collection.markers.push_back(marker_coordinates_with_distance);
    }
  }
  marker_coordinates_with_distance_collection.header.stamp = ros::Time::now();
  marker_coordinates_with_distance_collection.camera_height = image->height;
  marker_coordinates_with_distance_collection.camera_width = image->width;

  marker_coordinates_with_distance_collection_pub_.publish(marker_coordinates_with_distance_collection);
}

int main(int argc, char **argv)
{
  NodeMain<CalculateObjectDistanceAndAngle>(argc, argv, "CalculateObjectDistanceAndAngleNode");
}
