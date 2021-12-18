//#include <ros/ros.h>
//#include <sensor_msgs/Image.h>
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>


class ArucoMarkers {

  //Initialize the image object
  cv::Mat image;

  //Initialize the path string for the image (Only for debugging the tag detector alone)
  //TODO: Remove after implementing the code in the ROS node.
  std::string path;

  //Initialize vectors for the ID's of the markers and the marker corners
  std::vector<int> markerIds;
  std::vector<std::vector<cv::Point2f>> markerCorners;
  //std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
  //cv::Ptr<cv::aruco::DetectorParameters> parameters;

  //Define the dictionary to be detected
  cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
  //cv::aruco::detectMarkers(image, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);

public:

  void read_image(std::string path){
    //Read the file into the object
    //TODO: Replace hard-coded string for the path
    image = cv::imread(path);
  }

  void detect(){
    //Detect the ArUco tags
    cv::aruco::detectMarkers(image, dictionary, markerCorners, markerIds);
  }

  void display(){
    //Drawing the image with the detected tags
    cv::aruco::drawDetectedMarkers(image, markerCorners, markerIds);
    //Displaying the detected aruco tags in the image, in a window
    cv::namedWindow("Display Image", cv::WINDOW_NORMAL);
    cv::imshow("Display Image", image);
    cv::waitKey(0);
  }
};


int main(int argc, char *argv[])
{
  ArucoMarkers markers;

  //Uncomment the desired markers.read_image() line. Hard-coded path for the file left for ease of use.
  //markers.read_image(argv[1]);
  markers.read_image("/home/draugur/catkin_workspaces/my_ws/marker_dir/testing/4X4_50.png");

  markers.detect();
  markers.display();

  return(0);
}
