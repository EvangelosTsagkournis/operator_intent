#!/usr/bin/env python3

import os
import rospy
import rospkg
import numpy as np
import pandas as pd
import joblib
from sklearn.ensemble import RandomForestClassifier
from operator_intent_msgs.msg import marker_coordinates_with_distance_collection


class GoalInferenceKNN:
    def __init__(self, markers_set):
        self.max_log_size = 1000
        self.knn_number = 5
        self.state_log = pd.DataFrame()
        self.markers_set = markers_set
        print(self.markers_set)
        rospack = rospkg.RosPack()
        self.model = RandomForestClassifier
        # Loading the model saved in "${detect_poi}/ml_models/"
        self.model = joblib.load(os.path.join(rospack.get_path(
            "detect_poi"), "ml_models/marker_prediction_model.sav"))

        # Initialize ros stuff
        rospy.init_node('intent_inference_knn_node', anonymous=True)
        rospy.Subscriber("/aruco/persistent_marker_coordinates_with_distance_collection",
                         marker_coordinates_with_distance_collection, self.callback, queue_size=1)
        rospy.spin()

    def return_nearest_neighbors(self, current_state_df, nearest_neighbors_number):
        # TODO: Implement the logic for finding nearest neighbors, consider not using pd.Dataframe, instead using np.array

        data_columns = list(self.state_log.columns)
        
        ## Backup plan
        # data_columns = list(current_state_df)
        # data_columns.append("Goal")

        # Create np.array for the current state
        # current_state_array = current_state_df.drop('Goal', axis=1).to_numpy()
        current_state_array = current_state_df.to_numpy()

        # Normalize the current_state_
        current_state_array = current_state_array / current_state_array.max(axis=0)

        # Create np.array version of self.state_log
        # Dropping the 'Goal' column to allow for normalizing the rest of the values which are numeric
        state_log_array = self.state_log.drop('Goal', axis=1).to_numpy()

        # Normalize the array
        state_log_array = state_log_array / state_log_array.max(axis=0)

        # Calculate the squared euclidean distance between the current state and the log

        # Initialize empty 1D array
        euclidean_distance_squared = np.empty([self.max_log_size])

        # Calculate and append the value to the array
        for idx, i in enumerate(state_log_array):
            euclidean_distance_squared[idx] = np.sum(np.square(current_state_array - i))

        # Add the euclidean Goal array back into the state_log_array as well as the euclidean distance squared
        state_log_array = np.append(state_log_array, self.state_log.drop(self.state_log.columns.difference(['Goal']), 1).to_numpy().reshape(-1, 1), axis=1)
        state_log_array = np.append(state_log_array, euclidean_distance_squared.reshape(-1, 1), axis=1)

        # Sort the resulting state_log_array by ascending order by the last column (index -1: euclidean_distance_squared)

        state_log_array = state_log_array[state_log_array[:, -1].argsort()]

        # After sorting the array, pick an odd number to retrieve the inference results and pick the classification with the most entries,
        # and return a data structure with the number of entries closest to the current state

        df_nearest_neighbors = pd.DataFrame(state_log_array[:, :-1], columns = data_columns)

        # print("df_nearest_neighbors: ", df_nearest_neighbors)

        return df_nearest_neighbors

    def create_current_state_dataframe(self, persistent_marker_collection):
        current_state_df = pd.DataFrame()
        # Logic for marshalling the data in the appropriate format to pass to the model for prediction i.e self.model.predict(*)
        # At a later stage, consider the possibility of using self.model.predict_proba(*) to represent the probabilities of each label

        # For every marker in the persistent collection
        for i in persistent_marker_collection.markers:
            # If the marker_id is in the predefines set of markers we seek
            if str(i.marker_id) in self.markers_set:
                current_state_df["marker_{}_distance".format(i.marker_id)] = [
                    i.distance_mm]
                current_state_df["marker_{}_angle_radians".format(i.marker_id)] = [
                    i.angle_radians]
                current_state_df["marker_{}_approach_speed".format(i.marker_id)] = [
                    i.approaching_speed_meters_per_sec]

        # Rearranging the dataframe in the correct order to feed into the model
        current_state_df = current_state_df[[
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

        return current_state_df

    # Check if the size of the state log is > a value. If it is, pop the top value.
    def manage_log_size(self):
        # print("Before popping shape:", self.state_log.shape)
        # self.state_log.drop(self.state_log.index[0], inplace=True)
        self.state_log = self.state_log.iloc[1:, :]
        self.state_log.reset_index(drop=True, inplace=True)
        # print("Old log popped!")
        # print("After popping shape:", self.state_log.shape)

    def callback(self, persistent_marker_collection):
        current_state_df = self.create_current_state_dataframe(
            persistent_marker_collection)

        # print("Current state_log size:", len(self.state_log))
        # If a sufficient number of states has been recorded, use them to generate the goal
        if len(self.state_log) > self.max_log_size:
            # Check if the size of the state log is > a value. If it is, pop the oldest entry.
            self.manage_log_size()
            # Calculate goal and goal_probability and add to current_state_df
            nearest_states_df = self.return_nearest_neighbors(current_state_df, self.knn_number)
            # print("Log greater than", self.max_log_size)
            print("nearest_states_df prediction: ", nearest_states_df['Goal'].value_counts().idxmax())
            # TODO: do the goal calculation based on nearest_states_df returned
        
        current_goal = self.model.predict(current_state_df)
        current_goal_probability = self.model.predict_proba(
            current_state_df).max()
        current_state_df["Goal"] = current_goal
        print("Current goal inference and probability:",current_goal, "\n", current_goal_probability, "\n")
        # print("current_state_df:", current_state_df)

        # Append current state to state_log
        self.state_log = self.state_log.append(current_state_df)
        self.state_log.reset_index(drop=True, inplace=True)
        # print("self.state_log after append:", self.state_log)
        
        



if __name__ == "__main__":
    rospack = rospkg.RosPack()
    with open(os.path.join(rospack.get_path(
            "detect_poi"), "config/config.txt")) as f:
        markers_set = f.read().split(',')
    try:
        gi = GoalInferenceKNN(markers_set)
    except rospy.ROSInterruptException:
        pass
