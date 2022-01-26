#include <detect_aruco.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  std::string sub_topic = "/camera/rgb/image_raw";
  std::string pub_topic = "/image_converter/output_video";
  DetectAruco da(sub_topic, pub_topic);
  ros::spin();
  return 0;
}
