//#include <ros/ros.h>
//#include <sensor_msgs/Image.h>
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>

int main(int argc, char **argv)
{
  //Initialize the image object
  cv::Mat image, imageCopy;

  //A string variable to hold the absolute path of the image file where the tags are to be detected from
  std::string path = "/home/draugur/catkin_workspaces/my_ws/marker_dir/testing/4X4_50.png";

  //Read the file into the object
  //TODO: Replace hard-coded string for the path
  image = cv::imread(path);
  image.copyTo(imageCopy);

  //Initialize vectors for the ID's of the markers and the marker corners
  std::vector<int> markerIds;
  std::vector<std::vector<cv::Point2f>> markerCorners;
  //std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
  //cv::Ptr<cv::aruco::DetectorParameters> parameters;

  //Define the dictionary to be detected
  cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
 //cv::aruco::detectMarkers(image, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);

  //Detect the ArUco tags
  cv::aruco::detectMarkers(image, dictionary, markerCorners, markerIds);

  //Drawing the image with the detected tags
  cv::aruco::drawDetectedMarkers(imageCopy, markerCorners, markerIds);

  //Displaying the detected aruco tags in the image, in a window
  cv::namedWindow("Display Image", cv::WINDOW_NORMAL);
  cv::imshow("Display Image", imageCopy);
  cv::waitKey(0);
  return 0;

}
