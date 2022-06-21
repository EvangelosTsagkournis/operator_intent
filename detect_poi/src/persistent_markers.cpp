#include <ros/ros.h>

#include <operator_intent_msgs/point_2dc.h>
#include <operator_intent_msgs/marker.h>
#include <operator_intent_msgs/marker_collection.h>
#include <operator_intent_msgs/marker_coordinates_with_distance.h>
#include <operator_intent_msgs/marker_coordinates_with_distance_collection.h>

// Node template
#include "node_template.cpp"

class PersistentMarkers
{

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    operator_intent_msgs::marker_coordinates_with_distance_collection persistent_marker_collection;
    ros::Subscriber marker_collection_sub_;
    ros::Publisher persistent_marker_collection_pub_;
    void callBack(const operator_intent_msgs::marker_coordinates_with_distance_collectionConstPtr &msg);

public:
    PersistentMarkers(ros::NodeHandle nh, ros::NodeHandle pnh);
};

PersistentMarkers::PersistentMarkers(ros::NodeHandle nh, ros::NodeHandle pnh) : nh_(nh), pnh_(pnh)
{
    marker_collection_sub_ = nh_.subscribe<operator_intent_msgs::marker_coordinates_with_distance_collection>("/aruco/marker_coordinates_with_distance_collection", 1, &PersistentMarkers::callBack, this);
    persistent_marker_collection_pub_ = nh.advertise<operator_intent_msgs::marker_coordinates_with_distance_collection>("/aruco/persistent_marker_coordinates_with_distance_collection", 1);
    std::cout << "Sub & Pub done!" << std::endl;
}

void PersistentMarkers::callBack(const operator_intent_msgs::marker_coordinates_with_distance_collectionConstPtr &msg)
{
    std::cout << "Got in callback!" << std::endl;
    persistent_marker_collection.header = msg->header;
    std::cout << "Got in callback! 2" << std::endl;
    persistent_marker_collection.camera_height = msg->camera_height;
    persistent_marker_collection.camera_width = msg->camera_width;

    std::cout << "Size of msg markers: " << sizeof(msg->markers) / sizeof(msg->markers[0]) << std::endl;

    // Condition below won't let it enter, to be revised
    // if (sizeof(msg->markers) / sizeof(msg->markers[0]) != 0)
    // {
        std::cout << "Got in callback! 3" << std::endl;
        for (int i = 0; i < msg->markers.size(); i++)
        {
            std::cout << "Got in callback! 4" << std::endl;
            bool found = false;
            for (int j = 0; j < persistent_marker_collection.markers.size(); j++)
            {
                std::cout << "Got in callback! 5" << std::endl;

                if (msg->markers[i].marker_id == persistent_marker_collection.markers[j].marker_id)
                {
                    std::cout << "Got in callback! 6" << std::endl;
                    // Size of persistent_marker_collection.markers keeps growing, special attention to replace and NOT add
                    std::cout << "Size of persistent markers:" << persistent_marker_collection.markers.size() << std::endl;
                    persistent_marker_collection.markers[j] = msg->markers[i];
                    found = true;
                    break;
                }
            }
            if (!found)
            {
                std::cout << "Got in callback! 7" << std::endl;
                operator_intent_msgs::marker_coordinates_with_distance temp_marker;
                temp_marker.marker_id = msg->markers[i].marker_id;
                temp_marker.angle_radians = msg->markers[i].angle_radians;
                temp_marker.distance = msg->markers[i].distance;
                temp_marker.marker_pixel_x = msg->markers[i].marker_pixel_x;
                temp_marker.marker_pixel_y = msg->markers[i].marker_pixel_y;
                temp_marker.marker_world_x = msg->markers[i].marker_world_x;
                temp_marker.marker_world_y = msg->markers[i].marker_world_y;
                persistent_marker_collection.markers.push_back(temp_marker);
            }
        }
    // }
    persistent_marker_collection_pub_.publish(persistent_marker_collection);
    std::cout << "Published!" << std::endl;
}

int main(int argc, char **argv)
{
    NodeMain<PersistentMarkers>(argc, argv, "PersistentMarkersNode");
}