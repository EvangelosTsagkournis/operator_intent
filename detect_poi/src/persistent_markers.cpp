#include <ros/ros.h>

#include <operator_intent_msgs/point_2dc.h>
#include <operator_intent_msgs/marker.h>
#include <operator_intent_msgs/marker_collection.h>
#include <operator_intent_msgs/marker_coordinates_with_distance.h>
#include <operator_intent_msgs/marker_coordinates_with_distance_collection.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

// Node template
#include "node_template.cpp"

struct Point{
    double x;
    double y;
};

class PersistentMarkers
{

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    operator_intent_msgs::marker_coordinates_with_distance_collection persistent_marker_collection;
    // ros::Subscriber marker_collection_sub_;
    message_filters::Subscriber<operator_intent_msgs::marker_coordinates_with_distance_collection> marker_collection_sub_;
    message_filters::Subscriber<nav_msgs::Odometry> odometry_sub_;
    ros::Publisher persistent_marker_collection_pub_;
    void callBack(const operator_intent_msgs::marker_coordinates_with_distance_collectionConstPtr &marker_coordinates_with_distance_collection_msg,
                  const nav_msgs::OdometryConstPtr &odometry_msg);
    double calculateDistanceOfTwoPoints(Point &p1, Point &p2);
    void updatePersistentMarkerCollectionFromRobotPose(nav_msgs::OdometryConstPtr odometry_msg);

public:
    PersistentMarkers(ros::NodeHandle nh, ros::NodeHandle pnh);
};

PersistentMarkers::PersistentMarkers(ros::NodeHandle nh, ros::NodeHandle pnh) : nh_(nh), pnh_(pnh)
{
    persistent_marker_collection_pub_ = nh.advertise<operator_intent_msgs::marker_coordinates_with_distance_collection>("/aruco/persistent_marker_coordinates_with_distance_collection", 1);
    marker_collection_sub_.subscribe(nh_, "/aruco/marker_coordinates_with_distance_collection", 1);
    odometry_sub_.subscribe(nh_, "/husky_base_ground_truth", 1);
    typedef message_filters::sync_policies::ApproximateTime<operator_intent_msgs::marker_coordinates_with_distance_collection, nav_msgs::Odometry> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), marker_collection_sub_, odometry_sub_);
    sync.registerCallback(boost::bind(&PersistentMarkers::callBack, this, _1, _2));
    ros::spin();
}

double PersistentMarkers::calculateDistanceOfTwoPoints(Point &p1, Point &p2) {
    return sqrt(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2));
}

void PersistentMarkers::updatePersistentMarkerCollectionFromRobotPose(nav_msgs::OdometryConstPtr odometry_msg)
{
    Point robot_position = {odometry_msg->pose.pose.position.x, odometry_msg->pose.pose.position.y};
    for (int i = 0; i < persistent_marker_collection.markers.size(); i++)
    {
        // Update distance
        Point marker_position = {persistent_marker_collection.markers[i].marker_world_x, persistent_marker_collection.markers[i].marker_world_y};
        persistent_marker_collection.markers[i].distance_mm = calculateDistanceOfTwoPoints(robot_position, marker_position) * 1000;

        // Update angle
        double marker_world_angle_from_robot_base = atan2(marker_position.y - robot_position.y, marker_position.x - robot_position.x);
        if ((marker_world_angle_from_robot_base) < 0) {
            marker_world_angle_from_robot_base = 2*M_PI - marker_world_angle_from_robot_base;
        }

        tf2::Quaternion q(
            odometry_msg->pose.pose.orientation.x, 
            odometry_msg->pose.pose.orientation.y, 
            odometry_msg->pose.pose.orientation.z, 
            odometry_msg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        double robot_world_angle = yaw;

        if (robot_world_angle < 0) {
            robot_world_angle = 2*M_PI + robot_world_angle;
        }

        double marker_from_robot_angle = marker_world_angle_from_robot_base - robot_world_angle;
        if (marker_from_robot_angle > M_PI) {
            marker_from_robot_angle = 2*M_PI - marker_from_robot_angle;
        }
        else if (marker_from_robot_angle < -M_PI) {
            marker_from_robot_angle = 2*M_PI + marker_from_robot_angle;
        }
        persistent_marker_collection.markers[i].angle_radians = marker_from_robot_angle;
    }
    return;
}

void PersistentMarkers::callBack(
    const operator_intent_msgs::marker_coordinates_with_distance_collectionConstPtr &marker_coordinates_with_distance_collection_msg,
    const nav_msgs::OdometryConstPtr &odometry_msg)
{
    persistent_marker_collection.header = marker_coordinates_with_distance_collection_msg->header;
    persistent_marker_collection.camera_height = marker_coordinates_with_distance_collection_msg->camera_height;
    persistent_marker_collection.camera_width = marker_coordinates_with_distance_collection_msg->camera_width;

    // If the marker_coordinates_with_distance_collection_msg isn't empty aka no markers detected
    if (marker_coordinates_with_distance_collection_msg->markers.size() != 0)
    {
        // For each marker in marker_coordinates_with_distance_collection_msg
        for (int i = 0; i < marker_coordinates_with_distance_collection_msg->markers.size(); i++)
        {
            // Bool value to check if a match has been found
            bool found = false;
            // For each marker in persistent_marker_collection
            for (int j = 0; j < persistent_marker_collection.markers.size(); j++)
            {
                // If the marker_id in the marker_coordinates_with_distance_collection_msg matches the one in persistent_marker_collection
                if (marker_coordinates_with_distance_collection_msg->markers[i].marker_id == persistent_marker_collection.markers[j].marker_id)
                {
                    // And if the marker isn't out of bounds for the depth sensor
                    if (!(marker_coordinates_with_distance_collection_msg->markers[i].marker_world_x == 0.0 && marker_coordinates_with_distance_collection_msg->markers[i].marker_world_y == 0.0 && marker_coordinates_with_distance_collection_msg->markers[i].distance_mm == -1.0))
                    {
                        // Update the marker in the persistent_marker_collection with the one in the marker_coordinates_with_distance_collection_msg
                        persistent_marker_collection.markers[j] = marker_coordinates_with_distance_collection_msg->markers[i];
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
                operator_intent_msgs::marker_coordinates_with_distance temp_marker = marker_coordinates_with_distance_collection_msg->markers[i];
                persistent_marker_collection.markers.push_back(temp_marker);
            }
        }
    }
    updatePersistentMarkerCollectionFromRobotPose(odometry_msg);
    // Publish the current state of the persistent_marker_collection
    if (persistent_marker_collection.markers.size() != 0) {
        persistent_marker_collection_pub_.publish(persistent_marker_collection);
    }
}

int main(int argc, char **argv)
{
    NodeMain<PersistentMarkers>(argc, argv, "PersistentMarkersNode");
}