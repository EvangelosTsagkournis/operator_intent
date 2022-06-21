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
    persistent_marker_collection.header = msg->header;
    persistent_marker_collection.camera_height = msg->camera_height;
    persistent_marker_collection.camera_width = msg->camera_width;


    // If the msg isn't empty aka no markers detected
    if (msg->markers.size() != 0)
    {
        // For each marker in msg
        for (int i = 0; i < msg->markers.size(); i++)
        {
            // Bool value to check if a match has been found
            bool found = false;
            // For each marker in persistent_marker_collection
            for (int j = 0; j < persistent_marker_collection.markers.size(); j++)
            {
                // If the marker_id in the msg matches the one in persistent_marker_collection
                if (msg->markers[i].marker_id == persistent_marker_collection.markers[j].marker_id)
                {
                    // And if the marker isn't out of bounds for the depth sensor
                    if (!(msg->markers[i].marker_world_x == 0.0 && msg->markers[i].marker_world_y == 0.0 && msg->markers[i].distance == -1.0))
                    {
                        // Update the marker in the persistent_marker_collection with the one in the msg
                        persistent_marker_collection.markers[j] = msg->markers[i];
                    }
                    // Set the found to true and break
                    found = true;
                    break;
                }
            }
            // If a match has not been found:
            if (!found)
            {
                // Add a new marker to the persistent_marker_collection
                operator_intent_msgs::marker_coordinates_with_distance temp_marker = msg->markers[i];
                persistent_marker_collection.markers.push_back(temp_marker);
            }
        }
    }
    // Publish the current state of the persistent_marker_collection
    persistent_marker_collection_pub_.publish(persistent_marker_collection);
}

int main(int argc, char **argv)
{
    NodeMain<PersistentMarkers>(argc, argv, "PersistentMarkersNode");
}