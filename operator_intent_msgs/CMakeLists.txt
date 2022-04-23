cmake_minimum_required(VERSION 3.0.2)
project(operator_intent_msgs)


find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)

add_message_files(
  FILES
  point_2d.msg
  corner_array.msg
  marker_locations.msg
  pixel_with_distance.msg
  pixel_array.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  std_msgs message_runtime
)

