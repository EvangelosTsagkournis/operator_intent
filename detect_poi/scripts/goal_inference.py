#!/usr/bin/env python3

import os
import json
import rospy
import rospkg
import std_msgs.msg
from operator_intent_msgs.msg import operator_intent_inference
import pandas as pd
import joblib
from collections import Counter
from sklearn.ensemble import RandomForestClassifier
from operator_intent_msgs.msg import marker_coordinates_with_distance_collection


class GoalInference:
    def __init__(self, markers_set):
        self.markers_set = markers_set
        # Print the configuration for the markers to look for
        print("Markers set: ", self.markers_set)
        rospack = rospkg.RosPack()
        self.model = RandomForestClassifier
        # Loading the model saved in "${detect_poi}/ml_models/", it depends on the number of targets declared in the config.json file
        self.model = joblib.load(os.path.join(rospack.get_path(
            "detect_poi"), "ml_models/intent_inference_model_{}_targets.sav".format(len(self.markers_set))))

        # Initialize ros stuff
        rospy.init_node('operator_intent_inference_node', anonymous=True)
        self.persistent_marker_sub = rospy.Subscriber("/aruco/persistent_marker_coordinates_with_distance_collection",
                                                 marker_coordinates_with_distance_collection, self.callback, queue_size=1)
        self.operator_intent_pub = rospy.Publisher("/operator_intent_inference", operator_intent_inference, queue_size=1)
        rospy.spin()

    def is_subset(self, l1, l2):
        c1, c2 = Counter(l1), Counter(l2)
        return not c1 - c2
    
    def model_prediction(self):
        return self.model.predict()

    def callback(self, persistent_marker_collection):
        current_state_df = pd.DataFrame()

        persistent_marker_collection_id_list = list()
        for i in persistent_marker_collection.markers:
            persistent_marker_collection_id_list.append(i.marker_id)


        if self.is_subset(self.markers_set, persistent_marker_collection_id_list):
            # Logic for marshalling the data in the appropriate format to pass to the model for prediction i.e self.model.predict()
            # For every marker in the persistent collection
            for i in persistent_marker_collection.markers:
                # If the marker_id is in the predefines set of markers we seek
                if i.marker_id in self.markers_set:
                    current_state_df["marker_{}_distance".format(i.marker_id)] = [
                        i.distance_mm]
                    current_state_df["marker_{}_angle_radians".format(i.marker_id)] = [
                        i.angle_radians]
                    current_state_df["marker_{}_approach_speed".format(i.marker_id)] = [
                        i.approaching_speed_meters_per_sec]

            # Creating the input  data labels from the markers_set
            input_data_labels = list()

            for i in self.markers_set:
                input_data_labels.append("marker_{}_distance".format(i))
                input_data_labels.append("marker_{}_angle_radians".format(i))
                input_data_labels.append("marker_{}_approach_speed".format(i))

            # Move the dataframe columns to the appropriate place
            current_state_df = current_state_df[input_data_labels]

            # Make the prediction and get the probability for it
            prediction = self.model.predict(current_state_df)
            prediction_probability = self.model.predict_proba(current_state_df).max()

            # Construct the message to be sent
            operator_intent_inference_msg = operator_intent_inference()
            # Fill the fields of the message
            h = std_msgs.msg.Header()
            h.stamp = rospy.Time.now()
            operator_intent_inference_msg.header = h
            operator_intent_inference_msg.prediction = str(prediction)
            operator_intent_inference_msg.prediction_probability = prediction_probability

            # Publish the message
            self.operator_intent_pub.publish(operator_intent_inference_msg)




if __name__ == "__main__":
    rospack = rospkg.RosPack()
    with open(os.path.join(rospack.get_path("detect_poi"), "config/config.json")) as f:
        json_content = json.loads(f.read())
        markers_set = list()
        for marker in json_content["markers"]:
            markers_set.append(marker["id"])
        markers_set.sort()
    try:
        gi = GoalInference(markers_set)
    except rospy.ROSInterruptException:
        pass
