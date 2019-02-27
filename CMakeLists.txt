cmake_minimum_required(VERSION 2.8.3)
project(ez_pick_and_place)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  std_srvs
  moveit_msgs
  geometry_msgs
  moveit_commander
  manipulation_msgs
  message_generation
  grasp_planning_graspit_msgs
  household_objects_database_msgs
)

add_service_files(
   FILES
   EzSceneSetup.srv
   EzStartPlanning.srv
)

add_message_files(
  FILES
   EzModel.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS std_msgs geometry_msgs message_generation
)

