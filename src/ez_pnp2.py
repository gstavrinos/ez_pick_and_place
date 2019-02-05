#!/usr/bin/env python
import tf
import sys
import rospy
import moveit_commander

from grasp_planning_graspit_msgs.srv import AddToDatabase, LoadDatabaseModel
from ez_pick_and_place.srv import EzSceneSetup, EzStartPlanning
from manipulation_msgs.srv import GraspPlanning
from moveit_msgs.srv import GetPositionIK
from std_srvs.srv import Trigger

from ez_tools import EZToolSet
from ezpnp_sim_annealing import EzPnP

ez_tools = None

def main():
    global ez_tools, add_model_srv, load_model_srv, planning_srv

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("ez_pnp")

    ez_tools = EZToolSet()

    ez_tools.tf_listener = tf.TransformListener()
    ez_tools.moveit_scene = moveit_commander.PlanningSceneInterface()

    ez_tools.add_model_srv = rospy.ServiceProxy("/graspit_add_to_database", AddToDatabase)
    rospy.wait_for_service("/graspit_add_to_database")
    ez_tools.load_model_srv = rospy.ServiceProxy("/graspit_load_model", LoadDatabaseModel)
    rospy.wait_for_service("/graspit_load_model")
    ez_tools.planning_srv = rospy.ServiceProxy("/graspit_eg_planning", GraspPlanning)
    rospy.wait_for_service("/graspit_eg_planning")
    ez_tools.compute_ik_srv = rospy.ServiceProxy("/compute_ik", GetPositionIK)
    rospy.wait_for_service("/compute_ik")

    scene_srv = rospy.Service("ez_pnp/scene_setup", EzSceneSetup, ez_tools.scene_setup)
    start_srv = rospy.Service("ez_pnp/start_planning", EzStartPlanning, ez_tools.startPlanningCallback)
    stop_srv = rospy.Service("ez_pnp/stop_planning", Trigger, ez_tools.stopPlanning)

    #rospy.spin()

    # TODO
    while not rospy.is_shutdown():
        if len(ez_tools.grasp_poses) > 0:
            #annealer = EzPnP([], ez_tools)
            continue

main()
