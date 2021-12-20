#include <calculate_aruco_distance.h>


int main(int argc, char** argv)
{
  ros::init(argc, argv, "depth_node");
  CalculateArucoDistance cad;
  ros::spin();
  return 0;
}
