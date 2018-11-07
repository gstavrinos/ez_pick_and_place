#!/usr/bin/env python
import tf
import sys
import rospy
import moveit_commander

from grasp_planning_graspit_msgs.srv import AddToDatabase, LoadDatabaseModel, AddToDatabaseRequest, LoadDatabaseModelRequest
from ez_pick_and_place.srv import EzSceneSetup, EzSceneSetupResponse, EzStartPlanning
from geometry_msgs.msg import TransformStamped, PoseStamped, Pose
from manipulation_msgs.srv import GraspPlanning
from std_srvs.srv import Trigger

tf_listener = None
moveit_scene = None
planning_srv = None
add_model_srv = None
load_model_srv = None

ez_objects = dict()
ez_obstacles = dict()

def startPlanning(req):
    return True, ""

def stopPlanning(req):
    return True, ""

# Check if the input of the scene setup service is valid
def validSceneSetupInput(req):
    tmp = dict()
    info = ""
    error_codes = "0"
    if len(req.finger_joint_names) == 0:
        info = "Invalid service input: No finger_joint_names provided"
        error_codes = "1"
        return False, info, error_codes
    if req.gripper.name == "":
        info = "Invalid service input: No gripper name provided"
        error_codes = "6"
        return False, info, error_codes
    if req.gripper.graspit_file == "":
        info = "Invalid service input: No graspit filename provided for the gripper"
        error_codes = "5"
        return False, info, error_codes
    if req.pose_factor <= 0:
        info = "Invalid service input: pose_factor cannot be negative or zero"
        error_codes = "2"
        return False, info, error_codes

    for obj in req.objects:
        if obj.name == "":
            info = "Invalid service input: No object name provided"
            error_codes = "6"
            return False, info, error_codes
        if obj.name in tmp:
            info = "Invalid service input: Duplicate name: " + obj.name
            error_codes = "4"
            return False, info, error_codes
        else:
            tmp[obj.name] = 0
        if obj.graspit_file == "" and obj.moveit_file == "":
            info = "Invalid service input: No file provided for object: " + obj.name
            error_codes = "5"
            return False, info, error_codes
        if obj.pose.header.frame_id == "":
            info = "Invalid service input: No frame_id in PoseStamped message of object: " + obj.name
            error_codes = "3"
            return False, info, error_codes

    for obs in req.obstacles:
        if obs.name == "":
            info = "Invalid service input: No obstacle name provided"
            error_codes = "6"
            return False, info, error_codes
        if obs.name in tmp:
            info = "Invalid service input: Duplicate name: " + obs.name
            error_codes = "4"
            return False, info, error_codes
        else:
            tmp[obs.name] = 0
        if obs.graspit_file == "" and obs.moveit_file == "":
            info = "Invalid service input: No file provided for obstacle: " + obs.name
            error_codes = "5"
            return False, info, error_codes
        if obs.pose.header.frame_id == "":
            info = "Invalid service input: No frame_id in PoseStamped message of obstacle: " + obs.name
            error_codes = "3"
            return False, info, error_codes
    return True, info, error_codes

# Graspit bodies are always referenced relatively to the "world" frame
def fixItForGraspIt(obj, pose_factor):
    global tf_listener
    p = Pose()
    if obj.pose.header.frame_id == "world":
        p = obj.pose.pose
        p.position.x *= pose_factor
        p.position.y *= pose_factor
        p.position.z *= pose_factor
        #TODO orientation?
    else:
        transform = TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = obj.header.frame_id
        transform.child_frame_id = "ez_fix_it_for_grasp_it"
        transform.transform.translation.x = obj.pose.pose.position.x
        transform.transform.translation.y = obj.pose.pose.position.y
        transform.transform.translation.z = obj.pose.pose.position.z
        transform.transform.rotation.x = obj.pose.pose.orientation.x
        transform.transform.rotation.y = obj.pose.pose.orientation.y
        transform.transform.rotation.z = obj.pose.pose.orientation.z
        transform.transform.rotation.w = obj.pose.pose.orientation.w
        tf_listener.setTransform(transform, "fixItForGraspIt")

        trans, rot = tf_listener.lookupTransform("ez_fix_it_for_grasp_it", "world", rospy.Time(0))

        p.position.x = trans[0] * pose_factor
        p.position.y = trans[1] * pose_factor
        p.position.z = trans[2] * pose_factor
        p.orientation.x = rot[0]
        p.orientation.y = rot[1]
        p.orientation.z = rot[2]
        p.orientation.w = rot[3]

    return p

def scene_setup(req):
    global add_model_srv, load_model_srv, planning_srv, tf_listener, moveit_scene

    res = EzSceneSetupResponse()
    res.success = True

    valid, info, ec = validSceneSetupInput(req)

    try:
        if not valid:
            res.success = False
            res.info = info
            res.error_codes = ec
            return res
        # TODO handle graspit responses
        for obj in req.objects:
            # ------ Graspit world ------
            if obj.graspit_file != "":
                atd = AddToDatabaseRequest()
                atd.filename = obj.graspit_file
                atd.isRobot = False
                atd.asGraspable = True
                atd.modelName = obj.name
                response = add_model_srv(atd)

                objectID = response.modelID
                ez_objects[obj.name] = objectID

                loadm = LoadDatabaseModelRequest()
                loadm.model_id = objectID
                loadm.model_pose = fixItForGraspIt(obj, req.pose_factor)
                response = load_model_srv(loadm)
            # ---------------------------

            # ------ Moveit scene -------
            if obj.moveit_file != "":
                moveit_scene.add_mesh(obj.name, obj.pose, obj.moveit_file)
            # ---------------------------
        for obstacle in req.obstacles:
            # ------ Graspit world ------
            if obstacle.graspit_file != "":
                atd = AddToDatabaseRequest()
                atd.filename = obstacle.graspit_file
                atd.isRobot = False
                atd.asGraspable = False
                atd.modelName = obstacle.name
                response = add_model_srv(atd)
                obstacleID = response.modelID
                ez_obstacles[obstacle.name] = obstacleID

                loadm = LoadDatabaseModelRequest()
                loadm.model_id = obstacleID
                loadm.model_pose = fixItForGraspIt(obstacle, req.pose_factor)
                response = load_model_srv(loadm)
            # ---------------------------

            # ------ Moveit scene -------
            if obj.moveit_file != "":
                moveit_scene.add_mesh(obstacle.name, obstacle.pose, obstacle.moveit_file)
            # ---------------------------

        # ------ Graspit world ------
        atd = AddToDatabaseRequest()
        atd.filename = req.gripper.graspit_file
        atd.isRobot = True
        atd.asGraspable = False
        atd.modelName = req.gripper.name
        atd.jointNames = req.finger_joint_names
        response = add_model_srv(atd)
        robotID = response.modelID
        # ---------------------------

        return res

    except Exception as e:
        info = e
        error_codes = "-1"
        return False, info, error_codes

def main():
    global add_model_srv, load_model_srv, planning_srv, tf_listener, moveit_scene

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("ez_pnp")

    tf_listener = tf.TransformListener()
    moveit_scene = moveit_commander.PlanningSceneInterface()

    add_model_srv = rospy.ServiceProxy("/graspit_add_to_database", AddToDatabase)
    rospy.wait_for_service("/graspit_add_to_database")
    load_model_srv = rospy.ServiceProxy("/graspit_load_model", LoadDatabaseModel)
    rospy.wait_for_service("/graspit_load_model")
    planning_srv = rospy.ServiceProxy("/graspit_eg_planning", GraspPlanning)
    rospy.wait_for_service("/graspit_eg_planning")

    scene_srv = rospy.Service("ez_pnp/scene_setup", EzSceneSetup, scene_setup)
    start_srv = rospy.Service("ez_pnp/start_planning", EzStartPlanning, startPlanning)
    stop_srv = rospy.Service("ez_pnp/stop_planning", Trigger, stopPlanning)

    rospy.spin()

main()
