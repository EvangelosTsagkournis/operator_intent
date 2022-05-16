#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>

using namespace sensor_msgs;
using namespace message_filters;

void callback(const ImageConstPtr &image1, const ImageConstPtr &image2)
{
  // Solve all of perception here...
  std::cout << "Hello from the callback function! You've done it!" << std::endl;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "vision_node");

  ros::NodeHandle nh;
  message_filters::Subscriber<Image> image1_sub(nh, "/camera/rgb/image_raw", 1);
  message_filters::Subscriber<Image> image2_sub(nh, "/camera/depth/image_raw", 1);

  typedef sync_policies::ApproximateTime<Image, Image> MySyncPolicy;
  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image1_sub, image2_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2));

  ros::spin();

  return 0;
}