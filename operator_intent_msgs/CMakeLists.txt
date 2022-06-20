cmake_minimum_required(VERSION 3.0.2)
project(operator_intent_msgs)


find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)

add_message_files(
  FILES
  point_2dc.msg
  marker.msg
  marker_collection.msg
  marker_coordinates_with_distance.msg
  marker_coordinates_with_distance_collection.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  std_msgs message_runtime
)

