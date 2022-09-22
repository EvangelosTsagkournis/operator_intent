# The operator_intent package.

This package was developed on ROS Kinetic Kame. Its functionality is to detect Aruco markers via a Kinect RGBD camera, which are used as points of interest e.g goals the operator intends to approach. Using the robot's pose as a reference and the distance from each marker as well as the angle between the robot's pose and each goal, it is able to estimate the location of each marker. Making use of the angle, distance and the calculated approaching speed of the robot towards each marker, a machine-learning model infers the operator's intent.

## Installing
Clone the repository into your catkin workspace. Open a terminal, navigate to your catkin workspace and run:

```
catkin_make
```

## Running the Experiments
The experiments are contained in the operator_intent_experiments package. To run them, open a terminal and run:

```
roslaunch operator_experiments_launch <launch_file_name>
```
