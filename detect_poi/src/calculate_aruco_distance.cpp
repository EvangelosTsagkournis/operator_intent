#include <calculate_aruco_distance.h>

CalculateArucoDistance::CalculateArucoDistance() : it_(nh_)
{
  // Subscribe to input video feed and publish output video feed
  message_filters::Subscriber<operator_intent_msgs::marker_locations> marker_loc_sub(nh_, "/aruco/markers_loc", 1);
  message_filters::Subscriber<sensor_msgs::Image> depth_image_sub(nh_, "/camera/depth/image_raw", 1);

  typedef message_filters::sync_policies::ApproximateTime<operator_intent_msgs::marker_locations, sensor_msgs::Image> MySyncPolicy;

  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), marker_loc_sub, depth_image_sub);
  sync.registerCallback(boost::bind(&CalculateArucoDistance::callBack, this, _1, _2));

  /*
  image_sub_ = it_.subscribe("/camera/depth/image_raw", 1,
    &CalculateArucoDistance::callBack, this);
  image_pub_ = it_.advertise("/image_converter/output_video", 1);
  */
  cv::namedWindow(OPENCV_WINDOW);
}

CalculateArucoDistance::~CalculateArucoDistance() 
{
  cv::destroyWindow(OPENCV_WINDOW);
}

void CalculateArucoDistance::callBack(const operator_intent_msgs::marker_locationsConstPtr &markers, const sensor_msgs::ImageConstPtr &image)
{
  std::cout << "xd!" << std::endl;
}

void CalculateArucoDistance::callBack(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1); // sensor_msgs::image_encodings::BGR8 / TYPE_8UC1
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  //###################################################################################################
  // Normalize the image from [0, 255] to [0, 1]
  cv::Mat norm_image;
  cv_ptr->image.convertTo(norm_image, CV_32FC1, 1.0/5, 0);
  //cv::normalize(cv_ptr->image, norm_image, 0, 1, cv::NORM_MINMAX, CV_32FC1); //cv::NORM_MINMAX
  //for (int i = 0; i <)
  // Update GUI Window
  cv::imshow(OPENCV_WINDOW, norm_image);
  //cv::imshow(OPENCV_WINDOW, cv_ptr->image);
  cv::waitKey(3);

  // Log the upper left pixel value to the console, for debugging purposes
  //std::cout << norm_image.at<float>(0,0) << std::endl;

  // Output modified video stream
  //image_pub_.publish(cv_ptr->toImageMsg());
  /*
  #####################################################################################################
  ########################################## WORKS ####################################################
  #####################################################################################################
  // Normalize the image from [0, 255] to [0, 1]
  cv::Mat norm_image;
  cv_ptr->image.convertTo(norm_image, CV_32FC1, 1.0/5, 0);
  //cv::normalize(cv_ptr->image, norm_image, 0, 1, cv::NORM_MINMAX, CV_32FC1); //cv::NORM_MINMAX
  // Update GUI Window
  std::cout << norm_image.at<float>(0,0) << std::endl;
  cv::imshow(OPENCV_WINDOW, norm_image);
  //cv::imshow(OPENCV_WINDOW, cv_ptr->image);
  cv::waitKey(3);
  // Output modified video stream
  image_pub_.publish(cv_ptr->toImageMsg());
  #####################################################################################################
  ####################################### END OF WORKS ################################################
  #####################################################################################################
  */
}
