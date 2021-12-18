#include "CalculateArucoDistance.h"


int main(int argc, char** argv)
{
  ros::init(argc, argv, "depth_node");
  CalculateArucoDistance cad;
  ros::spin();
  return 0;
}
