#!/usr/bin/env python3

import csv
from math import asin, atan2, cos, sin, sqrt
import os
import os.path
import re
import sys
import json
import rospy
import rospkg
from operator_intent_msgs.msg import operator_intent_inference
import pandas as pd
import joblib
from sklearn.ensemble import RandomForestClassifier
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry

def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians

class GoalInferenceTests:
    def __init__(self, goal_a, goal_b, goal_c, rosbag_file_name, test_scenario, trial):
        self.goal_a = goal_a
        self.goal_b = goal_b
        self.goal_c = goal_c

        self.rosbag_file_name = os.path.splitext(rosbag_file_name)[0]
        self.test_scenario = test_scenario
        self.trial = trial

        self.counter=0

        # Print the locations for the three markers
        print("Goal a:\nx: {}\ny: {}".format(goal_a.x, goal_a.y))
        print("Goal b:\nx: {}\ny: {}".format(goal_b.x, goal_b.y))
        print("Goal c:\nx: {}\ny: {}".format(goal_c.x, goal_c.y))

        rospack = rospkg.RosPack()
        self.model = RandomForestClassifier

        # Loading the model saved in "${detect_poi}/ml_models/", it depends on the number of targets declared in the config.json file
        self.model = joblib.load(os.path.join(rospack.get_path(
            "detect_poi"), "ml_models/goal_prediction_model_3_goals.sav"))

        # Initialize ros stuff
        rospy.init_node('operator_intent_inference_node', anonymous=True)
        self.odometry_sub = rospy.Subscriber("husky_base_ground_truth", Odometry, self.callback, queue_size=2000)
        self.operator_intent_pub = rospy.Publisher("/operator_intent_inference", operator_intent_inference, queue_size=1)
        rospy.spin()
        

    def determinant(self, p1, p2):
        return p1.x * p2.y - p1.y * p2.x
        
    def dotProduct(self, p1, p2):
        return p1.x * p2.x + p1.y * p2.y

    def calculate_distance_from_each_goal(self, odometry_msg):
        distances = list()
        distances.append(sqrt((self.goal_a.x - odometry_msg.pose.pose.position.x)**2 + (self.goal_a.y - odometry_msg.pose.pose.position.y)**2)*1000)
        distances.append(sqrt((self.goal_b.x - odometry_msg.pose.pose.position.x)**2 + (self.goal_b.y - odometry_msg.pose.pose.position.y)**2)*1000)
        distances.append(sqrt((self.goal_c.x - odometry_msg.pose.pose.position.x)**2 + (self.goal_c.y - odometry_msg.pose.pose.position.y)**2)*1000)
        return distances

    def calculate_angle_from_each_goal(self, odometry_msg):

        orientation_q = odometry_msg.pose.pose.orientation
        (roll, pitch, yaw) = euler_from_quaternion(orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w)

        robot_world_direction_vector_normalized = Point()
        robot_world_direction_vector_normalized.x = cos(yaw)
        robot_world_direction_vector_normalized.y = sin(yaw)

        angles = list()

        # For goal_a
        marker_x_difference = self.goal_a.x - odometry_msg.pose.pose.position.x
        marker_y_difference = self.goal_a.y - odometry_msg.pose.pose.position.y

        marker_vector_magnitude = sqrt(marker_x_difference * marker_x_difference + marker_y_difference * marker_y_difference)

        marker_world_angle_from_robot_position_normalized = Point()
        marker_world_angle_from_robot_position_normalized.x = marker_x_difference / marker_vector_magnitude
        marker_world_angle_from_robot_position_normalized.y = marker_y_difference /marker_vector_magnitude

        angles.append(atan2(
            self.determinant(robot_world_direction_vector_normalized, marker_world_angle_from_robot_position_normalized),
            self.dotProduct(robot_world_direction_vector_normalized, marker_world_angle_from_robot_position_normalized)
        ))

        # For goal_b
        marker_x_difference = self.goal_b.x - odometry_msg.pose.pose.position.x
        marker_y_difference = self.goal_b.y - odometry_msg.pose.pose.position.y

        marker_vector_magnitude = sqrt(marker_x_difference * marker_x_difference + marker_y_difference * marker_y_difference)

        marker_world_angle_from_robot_position_normalized.x = marker_x_difference / marker_vector_magnitude
        marker_world_angle_from_robot_position_normalized.y = marker_y_difference /marker_vector_magnitude

        angles.append(atan2(
            self.determinant(robot_world_direction_vector_normalized, marker_world_angle_from_robot_position_normalized),
            self.dotProduct(robot_world_direction_vector_normalized, marker_world_angle_from_robot_position_normalized)
        ))

        # For goal_c
        marker_x_difference = self.goal_c.x - odometry_msg.pose.pose.position.x
        marker_y_difference = self.goal_c.y - odometry_msg.pose.pose.position.y

        marker_vector_magnitude = sqrt(marker_x_difference * marker_x_difference + marker_y_difference * marker_y_difference)

        marker_world_angle_from_robot_position_normalized.x = marker_x_difference / marker_vector_magnitude
        marker_world_angle_from_robot_position_normalized.y = marker_y_difference /marker_vector_magnitude

        angles.append(atan2(
            self.determinant(robot_world_direction_vector_normalized, marker_world_angle_from_robot_position_normalized),
            self.dotProduct(robot_world_direction_vector_normalized, marker_world_angle_from_robot_position_normalized)
        ))

        return angles
        

    def calculate_approaching_speed_towards_each_goal(self, odometry_msg):
        # TODO: Write the logic for the approaching speed!
        approaching_speeds = list()

        direction_vector = Point()

        #For goal_a
        direction_vector.x = self.goal_a.x - odometry_msg.pose.pose.position.x
        direction_vector.y = self.goal_a.y - odometry_msg.pose.pose.position.y

        vector_magnitude = sqrt(direction_vector.x * direction_vector.x + direction_vector.y * direction_vector.y)

        direction_vector.x = direction_vector.x / vector_magnitude
        direction_vector.y = direction_vector.y / vector_magnitude

        approaching_speeds.append(direction_vector.x * odometry_msg.twist.twist.linear.x + direction_vector.y * odometry_msg.twist.twist.linear.y)

        #For goal_b
        direction_vector.x = self.goal_b.x - odometry_msg.pose.pose.position.x
        direction_vector.y = self.goal_b.y - odometry_msg.pose.pose.position.y

        vector_magnitude = sqrt(direction_vector.x * direction_vector.x + direction_vector.y * direction_vector.y)

        direction_vector.x = direction_vector.x / vector_magnitude
        direction_vector.y = direction_vector.y / vector_magnitude

        approaching_speeds.append(direction_vector.x * odometry_msg.twist.twist.linear.x + direction_vector.y * odometry_msg.twist.twist.linear.y)

        #For goal_c
        direction_vector.x = self.goal_c.x - odometry_msg.pose.pose.position.x
        direction_vector.y = self.goal_c.y - odometry_msg.pose.pose.position.y

        vector_magnitude = sqrt(direction_vector.x * direction_vector.x + direction_vector.y * direction_vector.y)

        direction_vector.x = direction_vector.x / vector_magnitude
        direction_vector.y = direction_vector.y / vector_magnitude

        approaching_speeds.append(direction_vector.x * odometry_msg.twist.twist.linear.x + direction_vector.y * odometry_msg.twist.twist.linear.y)

        return approaching_speeds

    def write_goal_probabilities_to_files(self, seconds, nseconds, prediction_probabilities, prediction):
        goal_list = ["a", "b", "c"]
        with open("{}/Rosbags/BAGS/probabilities/{}.csv".format(os.path.expanduser('~'), self.rosbag_file_name), "a", newline='') as f:
            writer = csv.writer(f, delimiter=",")
            writer.writerow(
                [
                    "{}.{}".format(seconds, nseconds), 
                    prediction_probabilities[0][0], 
                    prediction_probabilities[0][1], 
                    prediction_probabilities[0][2],
                    prediction[0]
                ]
            )


    def callback(self, odometry_msg):

        seconds = odometry_msg.header.stamp.secs
        nseconds = odometry_msg.header.stamp.nsecs
        print("Time:{}.{}\n".format(seconds, nseconds))

        # Calculate distance from each goal
        distances = self.calculate_distance_from_each_goal(odometry_msg)
        # Calculate angle from each goal
        angles = self.calculate_angle_from_each_goal(odometry_msg)
        # Calculate approaching speed towards each goal
        approaching_speeds = self.calculate_approaching_speed_towards_each_goal(odometry_msg)

        self.counter+=1

        print("Counter:\n{}".format(self.counter))
        print("Distances:\n{}".format(distances))
        print("Angles:\n{}".format(angles))
        print("Approaching Speeds:\n{}".format(approaching_speeds))

        current_state_df = pd.DataFrame()

        goal_list = ["goal_a", "goal_b", "goal_c"]
        for idx, i in enumerate(goal_list):
            current_state_df["{}_distance".format(i)] = [distances[idx]]
            current_state_df["{}_angle_radians".format(i)] = [angles[idx]]
            current_state_df["{}_approach_speed".format(i)] = [approaching_speeds[idx]]

        # Make the prediction and get the probability for it
        prediction = self.model.predict(current_state_df)
        print("Prediction:\n{}".format(prediction))
        prediction_probabilities = self.model.predict_proba(current_state_df)
        print("Prediction probabilities:\n{}".format(prediction_probabilities))

        self.write_goal_probabilities_to_files(seconds, nseconds, prediction_probabilities, prediction)




if __name__ == "__main__":

    if len(sys.argv) >= 2:
        rosbag_file_name = sys.argv[1]
        test_scenario, trial = re.findall('\d+', rosbag_file_name)
        print(test_scenario, trial)
        if (test_scenario == '1' or test_scenario == '2' or test_scenario == '3'):
            
            rospack = rospkg.RosPack()
            with open(os.path.join(rospack.get_path("detect_poi"), "config/config.json")) as f:
                json_content = json.loads(f.read())

            # Goal a
            goal_a = Point()
            goal_a.x = json_content["test_scenarios"][test_scenario]["goal_a"]["x"]
            goal_a.y = json_content["test_scenarios"][test_scenario]["goal_a"]["y"]

            # Goal b
            goal_b = Point()
            goal_b.x = json_content["test_scenarios"][test_scenario]["goal_b"]["x"]
            goal_b.y = json_content["test_scenarios"][test_scenario]["goal_b"]["y"]

            # Goal c
            goal_c = Point()
            goal_c.x = json_content["test_scenarios"][test_scenario]["goal_c"]["x"]
            goal_c.y = json_content["test_scenarios"][test_scenario]["goal_c"]["y"]

            try:
                gi = GoalInferenceTests(goal_a, goal_b, goal_c, rosbag_file_name, test_scenario, trial)
            except rospy.ROSInterruptException:
                pass

        else:
            print("Argument is not valid, choose between 1-3")

