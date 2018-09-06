cmake_minimum_required(VERSION 2.8.3)
project(ez_pick_and_place)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
)

catkin_package()

include_directories()
