#include "DetectAruco.h"

/*
Preprocessor directives below:

EXPLICIT_CONSTRUCTOR:
    Handles whether the explicit constructor will be used.

EXPLICIT_DESTRUCTOR:
    Similar to EXPLICIT_CONSTRUCTOR, handles whether the explicit destructor will be used.

TERMINAL_INFO:
    Handles whether the amount of Aruco tags detected in the image will be printed to the terminal.
    It will also handle whether the corners of each of the detected marker corners will be printed to the terminal.
    For example, if 2 Aruco tags are detected in the image, you should expect an output like:

    Detected tags: 2
    Tag ID: 0
    {
        Point 0: x = 537, y = 28
        Point 1: x = 573, y = 28
        Point 2: x = 566, y = 68
        Point 3: x = 531, y = 68
    }
    Tag ID: 8
    {
        Point 0: x = 298, y = 27
        Point 1: x = 329, y = 27
        Point 2: x = 329, y = 60
        Point 3: x = 299, y = 60
    }
*/

#define EXPLICIT_CONSTRUCTOR 1
#define EXPLICIT_DESTRUCTOR 1
#define TERMINAL_INFO 0

void DetectAruco::imageCallback(const sensor_msgs::ImageConstPtr& msg)
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

    #if TERMINAL_INFO // PREPROCESSOR DIRECTIVE, if 1: gets compiled, if 0: does not 
    if (markerCorners.size() > 0) {
        std::cout << "Detected tags: " << markerCorners.size() << std::endl;
        for (unsigned long int i = 0; i < markerCorners.size(); i++)
        {
            // Print the first detected tag's id
            std::cout <<"Tag ID: " << markerIds[i] << std::endl << "{" << std::endl;;
            // Using a for loop 0...3 for each of the 4 corners of the tag
            for(unsigned long int j = 0; j < 4; j++)
            {
                //Printing out the pixels of the corners of the first tag detected in the image
                std::cout << std::setw(10) << "Point "<< j 
                    << ": x = " << markerCorners[i][j].x << ", " << "y = " << markerCorners[i][j].y << std::endl;
            }
        std::cout << "}" << std::endl;
        }
    }
    #endif

    std::vector<std::vector<cv::Point2f> > orthogonalMarkerCorners;
    // The code below will attempt to draw a bounding box containing the tag, if any tags were detected successfully
    if (markerCorners.size() > 0)
    {
        //std::vector<std::vector<cv::Point2f> > orthogonalMarkerCorners = markerCorners;
        orthogonalMarkerCorners = markerCorners;
        // Loop through all of the detected tags
        float min_x, min_y, max_x, max_y;
        for (unsigned long int i = 0; i < orthogonalMarkerCorners.size(); i++)
        {
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
            for (unsigned long int j = 0; j < 4; j++)
            {
                if (markerCorners[i][j].x < min_x) min_x = markerCorners[i][j].x;
                if (markerCorners[i][j].x > max_x) max_x = markerCorners[i][j].x;
                if (markerCorners[i][j].y < min_y) min_y = markerCorners[i][j].y;
                if (markerCorners[i][j].y > max_y) max_y = markerCorners[i][j].y;
            }

            // Quick and dirty way to assign corners to orthogonalMarkerCorners

            for (unsigned long int j = 0; j < 4; j++)
            {
                switch (j)
                {
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
            for (unsigned int j = 0; j < 3; j++)
            {
                cv::line(cv_ptr->image, orthogonalMarkerCorners[i][j], orthogonalMarkerCorners[i][j+1], color, thickness, cv::LINE_8);
            }
            cv::line(cv_ptr->image, orthogonalMarkerCorners[i][3], orthogonalMarkerCorners[i][0], color, thickness, cv::LINE_8);
        }
    }

    // Publish the marker corners to the topic "aruco/markers_loc"
    if (orthogonalMarkerCorners.size() > 0)
    {
        operator_intent_msgs::marker_locations marker_locations;
        for (unsigned long int i = 0; i < orthogonalMarkerCorners.size(); i++){
            operator_intent_msgs::corner_array corner_array;
            corner_array.markerId = markerIds[i];
            for (unsigned long int j = 0; j < 4; j++)
            {
                operator_intent_msgs::point2d point2d;
                point2d.x = orthogonalMarkerCorners[i][j].x;
                point2d.y = orthogonalMarkerCorners[i][j].y;
                corner_array.corner_points[j] = point2d;
            }
            marker_locations.header.stamp = ros::Time::now();
            marker_locations.n_markers = orthogonalMarkerCorners.size();
            marker_locations.markers.push_back(corner_array);

        }
        markers_loc_pub.publish(marker_locations);
    }

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);

    // Output modified video stream
    //image_pub_.publish(cv_ptr->toImageMsg());

}

#if EXPLICIT_CONSTRUCTOR
DetectAruco::DetectAruco(std::string sub_rgb_image_topic, std::string pub_topic)
    :it_(nh_)
{
    m_sub_rgb_image_topic = sub_rgb_image_topic;
    m_pub_topic = pub_topic;
    // Subscribe to input video feed and publish output video feed
    image_sub_ = it_.subscribe(m_sub_rgb_image_topic, 1,
      &DetectAruco::imageCallback, this);
    image_pub_ = it_.advertise(m_pub_topic, 1);
    markers_loc_pub = nh_.advertise<operator_intent_msgs::marker_locations>("aruco/markers_loc", 1);

    cv::namedWindow(OPENCV_WINDOW);
}
#endif

#if EXPLICIT_DESTRUCTOR
DetectAruco::~DetectAruco()
{
    cv::destroyWindow(OPENCV_WINDOW);
}
#endif
