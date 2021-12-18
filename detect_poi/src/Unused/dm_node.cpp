//This is the marker detection node

#define DESTRUCTOR 0
// Image conversion from ROS to CV format libraries
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


class Detect_Markers
{
private:
  const std::string OPENCV_WINDOW = "Aruco Marker Detection";
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

  // Declare vectors for the ID's of the markers and the marker corners
  std::vector<int> markerIds;
  std::vector<std::vector<cv::Point2f>> markerCorners;

  // The below are commented out, as they are only for the rejected candidates for tags
  // std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
  // cv::Ptr<cv::aruco::DetectorParameters> parameters;

  // Define the dictionary to be detected
  cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);

  void imageCallback(const sensor_msgs::ImageConstPtr& msg)
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
    cv::aruco::detectMarkers(cv_ptr->image, dictionary, markerCorners, markerIds);
    // Draw the bounding box around the detected tags using the provided drawDetectedMarkers method
    // Commented out for debugging
    /*
    cv::aruco::drawDetectedMarkers(cv_ptr->image, markerCorners, markerIds);
    */
    // Checking if the markerCorners array is empty, if it is, we skip it (keep in mind the
    // dimensions of the markerCorners array is Nx4, where N is the number of tags detected)
    if (markerCorners.size() > 0) {
      std::cout << "Detected tags: " << markerCorners.size() << std::endl;
      for (unsigned long int i = 0; i < markerCorners.size(); i++){
        // Print the first detected tag's id
        std::cout <<"Tag ID: " << markerIds[i] << std::endl << "{" << std::endl;;
        // Using a for loop 0...3 for each of the 4 corners of the tag
        for(unsigned long int j = 0; j < 4; j++){
          //Printing out the pixels of the corners of the first tag detected in the image
          std::cout << " Point "<< j << ": x = " << markerCorners[i][j].x << " , " << "y = " << markerCorners[i][j].y << std::endl;
        }
        std::cout << "}" << std::endl;
      }
    }

    // The code below will attempt to draw a bounding box containing the tag, if any tags were detected successfully
    if (markerCorners.size() > 0) {
      std::vector<std::vector<cv::Point2f>> orthogonalMarkerCorners = markerCorners;
      // Loop through all of the detected tags
      float min_x, min_y, max_x, max_y;
      for (unsigned long int i = 0; i < orthogonalMarkerCorners.size(); i++){
        // Initialize the minimum and maximum values of the corners to the marker's first corner values
        min_x = markerCorners[i][0].x;
        min_y = markerCorners[i][0].y;
        max_x = markerCorners[i][0].x;
        max_y = markerCorners[i][0].y;

        // Check for the minimum and maximum values of x and y of all the corners, in order to draw an orthogonal bounding box.

        /* The reason for the orthogonal bounding box instead of the one provided by the drawDetectedMarkers method
         * offered in the aruco library is for the sole purpose of iterating through the pixels of the image
         * for the depth information. The drawDetectedMarkers will provide boundaries for the
         * markers, however it might not be orthogonal and that introduces problems for the logic. */
        for (unsigned long int j = 0; j < 4; j++){
          if (markerCorners[i][j].x < min_x) min_x = markerCorners[i][j].x;
          if (markerCorners[i][j].x > max_x) max_x = markerCorners[i][j].x;
          if (markerCorners[i][j].y < min_y) min_y = markerCorners[i][j].y;
          if (markerCorners[i][j].y > max_y) max_y = markerCorners[i][j].y;
        }

        // Quick and dirty way to assign corners to orthogonalMarkerCorners

        for (unsigned long int j = 0; j < 4; j++){
          switch (j) {
            case 0:
              orthogonalMarkerCorners[i][j].x = min_x;
              orthogonalMarkerCorners[i][j].y = min_y;
              break;
            case 1:
              orthogonalMarkerCorners[i][j].x = max_x;
              orthogonalMarkerCorners[i][j].y = min_y;
              break;
            case 2:
              orthogonalMarkerCorners[i][j].x = max_x;
              orthogonalMarkerCorners[i][j].y = max_y;
              break;
            case 3:
              orthogonalMarkerCorners[i][j].x = min_x;
              orthogonalMarkerCorners[i][j].y = max_y;
              break;
          }
        }

        // Line Thickness for drawing the borders
        int thickness = 1;

        // Configure the color for the lines
        cv::Scalar color = cv::Scalar(0, 255, 0);

        // Loop through the orthogonalMarkerCorners and draw the bounding boxes
        for (unsigned int j = 0; j < 3; j++){
          cv::line(cv_ptr->image, orthogonalMarkerCorners[i][j], orthogonalMarkerCorners[i][j+1], color, thickness, cv::LINE_8);
        }
        cv::line(cv_ptr->image, orthogonalMarkerCorners[i][3], orthogonalMarkerCorners[i][0], color, thickness, cv::LINE_8);
      }
    }

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }

public:
  Detect_Markers()
    :it_(nh_)
  {
    // Subscribe to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1,
      &Detect_Markers::imageCallback, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW);
  }

  #if DESTRUCTOR
  ~Detect_Markers()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }
  #endif
//  void drawTag(unsigned int &tagg, cv::Mat &image){
//    int thickness = 1;
//    for (unsigned int j = 0; j < 3; j++){
//      cv::line(image, orthogonalMarkerCorners[tagg][j], orthogonalMarkerCorners[tagg][j+1], cv::Scalar(0, 255, 0), thickness, cv::LINE_8);
//    }
//    cv::line(image, orthogonalMarkerCorners[tagg][3], orthogonalMarkerCorners[tagg][0], cv::Scalar(0, 255, 0), thickness, cv::LINE_8);
//  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  Detect_Markers dm;
  ros::spin();
  return 0;
}
