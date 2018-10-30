#!/usr/bin/env python
import tf
import rospy
import moveit_commander

from grasp_planning_graspit_msgs.srv import AddToDatabase, LoadDatabaseModel, AddToDatabaseRequest, LoadDatabaseModelRequest
from ez_pick_and_place.srv import EzSceneSetup, EzSceneSetupResponse
from manipulation_msgs.srv import GraspPlanning
from geometry_msgs.msg import PoseStamped, Pose

planning_srv = None
add_model_srv = None
load_model_srv = None

ez_objects = dict()
ez_obstacles = dict()

def validInput(req):
    info = ""
    error_codes = "0"
    if len(req.finger_joint_names) == 0:
        info = "Invalid service input: No finger_joint_names provided"
        error_codes = "1"
        return False, info, error_codes
    if req.pose_factor <= 0:
        info = "Invalid service input: pose_factor cannot be negative or zero"
        error_codes = "2"
        return False, info, error_codes
    for obj in req.objects:
        if obj.pose.header.frame_id == '':
            info = "Invalid service input: No frame_id in PoseStamped message"
            error_codes = "3"
            return False, info, error_codes
    for obs in req.obstacles:
        if obs.pose.header.frame_id == '':
            info = "Invalid service input: No frame_id in PoseStamped message"
            error_codes = "3"
            return False, info, error_codes
    return True, info, error_codes

def scene_setup(req):
    global add_model_srv, load_model_srv, planning_srv

    res = EzSceneSetupResponse()

    valid, info, ec = validInput(req)

    if not valid:
        res.success = False
        res.info = info
        res.error_codes = ec
        return res

    for obj in req.objects:
        atd = AddToDatabaseRequest()
        atd.filename =obj.file
        atd.isRobot = False
        atd.asGraspable = True
        atd.modelName = obj.name
        response = add_model_srv(atd)

        objectID = response.modelID
        ez_objects[obj.name] = objectID

        loadm = LoadDatabaseModelRequest()
        loadm.model_id = objectID
        p = obj.pose.pose
        p.position.x *= req.pose_factor
        p.position.y *= req.pose_factor
        p.position.z *= req.pose_factor
        #TODO orientation
        loadm.model_pose = p
        response = load_model_srv(loadm)
    for obstacle in req.obstacles:
        atd = AddToDatabaseRequest()
        atd.filename = obstacle.file
        atd.isRobot = False
        atd.asGraspable = False
        atd.modelName = obstacle.name
        response = add_model_srv(atd)
        obstacleID = response.modelID
        ez_obstacles[obstacle.name] = obstacleID

        loadm = LoadDatabaseModelRequest()
        loadm.model_id = obstacleID
        p = obstacle.pose.pose
        p.position.x *= req.pose_factor
        p.position.y *= req.pose_factor
        p.position.z *= req.pose_factor
        #TODO orientation
        loadm.model_pose = p
        response = load_model_srv(loadm)

    atd = AddToDatabaseRequest()
    atd.filename = req.gripper.file
    atd.isRobot = True
    atd.asGraspable = False
    atd.modelName = req.gripper.name
    atd.jointNames = req.finger_joint_names
    response = add_model_srv(atd)
    robotID = response.modelID

    return res

def main():
    global add_model_srv, load_model_srv, planning_srv

    rospy.init_node("ez_pnp")

    add_model_srv = rospy.ServiceProxy("/graspit_add_to_database", AddToDatabase)
    rospy.wait_for_service("/graspit_add_to_database")
    load_model_srv = rospy.ServiceProxy("/graspit_load_model", LoadDatabaseModel)
    rospy.wait_for_service("/graspit_load_model")
    planning_srv = rospy.ServiceProxy("/graspit_eg_planning", GraspPlanning)
    rospy.wait_for_service("/graspit_eg_planning")

    scene_srv = rospy.Service("ez_pnp/scene_setup", EzSceneSetup, scene_setup)

    rospy.spin()

main()
