#!/usr/bin/env python3

import os
import json
import rospy
import rospkg
import numpy as np
import pandas as pd
import joblib
from sklearn.ensemble import RandomForestClassifier
from operator_intent_msgs.msg import marker_coordinates_with_distance_collection


class GoalInference:
    def __init__(self, markers_set):
        self.markers_set = markers_set
        
        # Print the configuration for the markers to look for
        print("Markers set: ", self.markers_set)
        rospack = rospkg.RosPack()
        self.model = RandomForestClassifier
        # Loading the model saved in "${detect_poi}/ml_models/"
        self.model = joblib.load(os.path.join(rospack.get_path(
            "detect_poi"), "ml_models/marker_prediction_model.sav"))

        # Initialize ros stuff
        rospy.init_node('intent_inference_node', anonymous=True)
        rospy.Subscriber("/aruco/persistent_marker_coordinates_with_distance_collection",
                         marker_coordinates_with_distance_collection, self.callback, queue_size=1)
        rospy.spin()

    def callback(self, persistent_marker_collection):
        prediction_df = pd.DataFrame()
        # Logic for marshalling the data in the appropriate format to pass to the model for prediction i.e self.model.predict(*)
        # At a later stage, consider the possibility of using self.model.predict_proba(*) to represent the probabilities of each label

        # For every marker in the persistent collection
        for i in persistent_marker_collection.markers:
            # If the marker_id is in the predefines set of markers we seek
            if str(i.marker_id) in self.markers_set:
                prediction_df["marker_{}_distance".format(i.marker_id)] = [i.distance_mm]
                prediction_df["marker_{}_angle_radians".format(i.marker_id)] = [i.angle_radians]
                prediction_df["marker_{}_approach_speed".format(i.marker_id)] = [i.approaching_speed_meters_per_sec]
        
        # Rearranging the dataframe in the correct order to feed into the model
        prediction_df = prediction_df[[
            "marker_0_distance",
            "marker_0_angle_radians",
            "marker_0_approach_speed",
            "marker_8_distance",
            "marker_8_angle_radians",
            "marker_8_approach_speed",
            "marker_15_distance",
            "marker_15_angle_radians",
            "marker_15_approach_speed"
        ]]

        print(self.model.predict(prediction_df), "\n", self.model.predict_proba(prediction_df), "\n")

    def model_prediction(self):
        return self.model.predict()


if __name__ == "__main__":
    rospack = rospkg.RosPack()
    with open(os.path.join(rospack.get_path("detect_poi"), "config/config.json")) as f:
        json_content = json.loads(f.read())
        markers_set = list()
        for marker in json_content["markers"]:
            markers_set.append(marker["id"])
    try:
        gi = GoalInference(markers_set)
    except rospy.ROSInterruptException:
        pass
