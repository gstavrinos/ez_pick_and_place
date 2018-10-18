cmake_minimum_required(VERSION 2.8.3)
project(ez_pick_and_place)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  manipulation_msgs
  message_generation
  household_objects_database_msgs
)

add_service_files(
   FILES
   GraspitPlanning.srv
)

generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS std_msgs geometry_msgs message_generation
)

