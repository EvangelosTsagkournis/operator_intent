#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
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

#include "node_template.cpp"


/////////////////////////////////////////////////////////////////////////////
// Helper material to find whether a point is inside the area of an object //
/////////////////////////////////////////////////////////////////////////////

// Define Infinite (Using INT_MAX caused overflow problems) 
#define INF 10000

struct Point 
{ 
    int x; 
    int y; 
}; 

// Given three collinear points p, q, r, the function checks if 
// point q lies on line segment 'pr' 
bool onSegment(Point p, Point q, Point r) 
{ 
  if (q.x <= std::max(p.x, r.x) && q.x >= std::min(p.x, r.x) && q.y <= std::max(p.y, r.y) && q.y >= std::min(p.y, r.y))
    return true; 
  return false; 
} 
 
// To find orientation of ordered triplet (p, q, r). 
// The function returns following values 
// 0 --> p, q and r are collinear 
// 1 --> Clockwise 
// 2 --> Counterclockwise 
int orientation(Point p, Point q, Point r) 
{ 
    int val = (q.y - p.y) * (r.x - q.x) - 
            (q.x - p.x) * (r.y - q.y); 
 
    if (val == 0) return 0; // collinear 
    return (val > 0)? 1: 2; // clock or counterclock wise 
} 
 
// The function that returns true if line segment 'p1q1' 
// and 'p2q2' intersect. 
bool doIntersect(Point p1, Point q1, Point p2, Point q2) 
{ 
    // Find the four orientations needed for general and 
    // special cases 
    int o1 = orientation(p1, q1, p2); 
    int o2 = orientation(p1, q1, q2); 
    int o3 = orientation(p2, q2, p1); 
    int o4 = orientation(p2, q2, q1); 
 
    // General case 
    if (o1 != o2 && o3 != o4) 
        return true; 
 
    // Special Cases 
    // p1, q1 and p2 are collinear and p2 lies on segment p1q1 
    if (o1 == 0 && onSegment(p1, p2, q1)) return true; 
 
    // p1, q1 and p2 are collinear and q2 lies on segment p1q1 
    if (o2 == 0 && onSegment(p1, q2, q1)) return true; 
 
    // p2, q2 and p1 are collinear and p1 lies on segment p2q2 
    if (o3 == 0 && onSegment(p2, p1, q2)) return true; 
 
    // p2, q2 and q1 are collinear and q1 lies on segment p2q2 
    if (o4 == 0 && onSegment(p2, q1, q2)) return true; 
 
    return false; // Doesn't fall in any of the above cases 
} 
 
// Returns true if the point p lies inside the polygon[] with n vertices 
bool isInside(Point polygon[], Point p) 
{ 
    int n = sizeof(polygon) / sizeof(polygon[0]);
    // There must be at least 3 vertices in polygon[] 
    if (n < 3) return false; 
 
    // Create a point for line segment from p to infinite 
    Point extreme = {INF, p.y}; 
 
    // Count intersections of the above line with sides of polygon 
    int count = 0, i = 0; 
    do
    { 
        int next = (i+1)%n; 
 
        // Check if the line segment from 'p' to 'extreme' intersects 
        // with the line segment from 'polygon[i]' to 'polygon[next]' 
        if (doIntersect(polygon[i], polygon[next], p, extreme)) 
        { 
            // If the point 'p' is collinear with line segment 'i-next', 
            // then check if it lies on segment. If it lies, return true, 
            // otherwise false 
            if (orientation(polygon[i], p, polygon[next]) == 0) 
            return onSegment(polygon[i], p, polygon[next]); 
 
            count++; 
        } 
        i = next; 
    } while (i != 0); 
 
    // Return true if count is odd, false otherwise 
    return count&1; // Same as (count%2 == 1) 
} 

////////////////////////////////////////////////////////////////////////////////////
// End of helper material to find whether a point is inside the area of an object //
////////////////////////////////////////////////////////////////////////////////////

//static const std::string OPENCV_WINDOW = "Image Window";

class CalculateArucoDistance
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  message_filters::Subscriber<operator_intent_msgs::marker_locations> marker_loc_sub;
  message_filters::Subscriber<operator_intent_msgs::marker_locations> orthogonal_marker_loc_sub;
  //message_filters::Subscriber<sensor_msgs::Image> depth_image_sub;

  typedef image_transport::SubscriberFilter ImageSubscriber;
  ImageSubscriber depth_image_sub;

public:
  CalculateArucoDistance();
  CalculateArucoDistance(ros::NodeHandle, ros::NodeHandle);
  ~CalculateArucoDistance();

  void callBack(const operator_intent_msgs::marker_locationsConstPtr &marker_locations, const operator_intent_msgs::marker_locationsConstPtr &orthogonal_marker_locations, const sensor_msgs::ImageConstPtr &image);
};

CalculateArucoDistance::CalculateArucoDistance(ros::NodeHandle nh, ros::NodeHandle pnh) :
  it_(nh_),
  depth_image_sub(it_, "camera/depth/image_raw", 1)
{
  // Subscribe to input video feed and publish output video feed
  marker_loc_sub.subscribe(nh_, "/aruco/markers_loc", 1);
  orthogonal_marker_loc_sub.subscribe(nh_, "/aruco/orthogonal_markers_loc", 1);
  //depth_image_sub.subscribe(nh_, "/camera/depth/image_raw", 1);

  //Checkpoint #0
  std::cout << "Checkpoint #0" << std::endl;

  typedef message_filters::sync_policies::ApproximateTime<operator_intent_msgs::marker_locations, operator_intent_msgs::marker_locations, sensor_msgs::Image> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), marker_loc_sub, orthogonal_marker_loc_sub, depth_image_sub);
  sync.registerCallback(boost::bind(&CalculateArucoDistance::callBack, this, _1, _2, _3));
  ros::spin();

  /*
  image_sub_ = it_.subscribe("/camera/depth/image_raw", 1,
    &CalculateArucoDistance::callBack, this);
  image_pub_ = it_.advertise("/image_converter/output_video", 1);
  */
}

CalculateArucoDistance::~CalculateArucoDistance() 
{
}

void CalculateArucoDistance::callBack(
  const operator_intent_msgs::marker_locationsConstPtr &marker_locations, 
  const operator_intent_msgs::marker_locationsConstPtr &orthogonal_marker_locations, 
  const sensor_msgs::ImageConstPtr &image
  )
{
  // Checkpoint #1
  std::cout << "Checkpoint #1" << std::endl;
  cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
  
  // Write the logic for the depth image average calculation
  std::cout << "xd!" << std::endl;
  //std::cout << marker_locations->markers[0].corner_points[0].x << std::endl;
  if (orthogonal_marker_locations->markers.size() > 0){
    // For each marker [0...n]:
    for (unsigned long int i = 0; i <= orthogonal_marker_locations->markers.size(); i++) {
      // Checkpoint #2
      std::cout << "Checkpoint #2" << std::endl;
      double sum = 0;
      unsigned long count = 0;
      Point marker[4];
      int min_x;
      int min_y;
      int max_x;
      int max_y;
      // Set the minimum and maximum values of x and y to the coordinates of the current marker's first corner
      // Additionally, set the marker[0]'s x and y to the first corner of the current marker
      min_x, max_x, marker[0].x = orthogonal_marker_locations->markers[i].corner_points[0].x;
      min_y, max_y, marker[0].y= orthogonal_marker_locations->markers[i].corner_points[0].y;
      // Additionally, set the marker[0]'s x and y to the first corner of the current marker

      // For the four corners of each marker [0...3]:
      for (unsigned long int j = 1; j <= orthogonal_marker_locations->markers[i].corner_points.size(); j++) {
        marker[j].x = marker_locations->markers[i].corner_points[j].x;
        marker[j].y = marker_locations->markers[i].corner_points[j].y;
        if (min_x > orthogonal_marker_locations->markers[i].corner_points[j].x) min_x = orthogonal_marker_locations->markers[i].corner_points[j].x;
        if (max_x < orthogonal_marker_locations->markers[i].corner_points[j].x) max_x = orthogonal_marker_locations->markers[i].corner_points[j].x;
        if (min_y > orthogonal_marker_locations->markers[i].corner_points[j].y) min_y = orthogonal_marker_locations->markers[i].corner_points[j].y;
        if (max_y < orthogonal_marker_locations->markers[i].corner_points[j].y) max_y = orthogonal_marker_locations->markers[i].corner_points[j].y;
      }

      Point pixel;
      for (int j = min_x; j <= max_x; j++) {
        for (int k = min_y; k <= max_y; k++) {
          pixel.x = j;
          pixel.y = k;
          if (isInside(marker, pixel)){
            sum += cv_ptr->image.at<double>(pixel.x, pixel.y);
            count++;
          }
        }
      }
      std::cout << sum/count << std::endl;
    }
  }
}

int main(int argc, char **argv) {
  NodeMain<CalculateArucoDistance>(argc, argv, "CalculateArucoDistanceNode");
}
