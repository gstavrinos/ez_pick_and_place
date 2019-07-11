#!/usr/bin/env python
import sys
import rospy
import tf2_ros
import moveit_commander

from grasp_planning_graspit_msgs.srv import AddToDatabase, LoadDatabaseModel
from ez_pick_and_place.srv import EzSceneSetup, EzStartPlanning
from moveit_msgs.srv import GraspPlanning, GetPositionIK

from ez_tools import EZToolSet

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("ez_pnp")

    ez_tools = EZToolSet()

    ez_tools.debug = rospy.get_param("/ez_pnp/debug", False)

    ez_tools.moveit_scene = moveit_commander.PlanningSceneInterface()
    ez_tools.tf2_buffer = tf2_ros.Buffer()
    ez_tools.tf2_listener = tf2_ros.TransformListener(ez_tools.tf2_buffer)

    ez_tools.add_model_srv = rospy.ServiceProxy("/graspit_add_to_database", AddToDatabase)
    rospy.wait_for_service("/graspit_add_to_database")
    ez_tools.load_model_srv = rospy.ServiceProxy("/graspit_load_model", LoadDatabaseModel)
    rospy.wait_for_service("/graspit_load_model")
    ez_tools.planning_srv = rospy.ServiceProxy("/graspit_eg_planning", GraspPlanning)
    rospy.wait_for_service("/graspit_eg_planning")
    ez_tools.compute_ik_srv = rospy.ServiceProxy("/compute_ik", GetPositionIK)
    rospy.wait_for_service("/compute_ik")

    start_srv = rospy.Service("ez_pnp/start_planning", EzStartPlanning, ez_tools.startPlanning)
    scene_srv = rospy.Service("ez_pnp/scene_setup", EzSceneSetup, ez_tools.sceneSetup)

    rospy.spin()

main()
