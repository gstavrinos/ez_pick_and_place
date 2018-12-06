#!/usr/bin/env python
import tf
import sys
import time
import rospy
import moveit_commander

from grasp_planning_graspit_msgs.srv import AddToDatabase, LoadDatabaseModel, AddToDatabaseRequest, LoadDatabaseModelRequest
from ez_pick_and_place.srv import EzSceneSetup, EzSceneSetupResponse, EzStartPlanning
from household_objects_database_msgs.msg import DatabaseModelPose
from geometry_msgs.msg import TransformStamped, PoseStamped, Pose
from manipulation_msgs.msg import GraspableObject
from manipulation_msgs.srv import GraspPlanning
from std_srvs.srv import Trigger

tf_listener = None
moveit_scene = None
planning_srv = None
add_model_srv = None
load_model_srv = None

ez_objects = dict()
ez_obstacles = dict()

gripper_name = None

keep_planning = True

def stopPlanning(req):
    keep_planning = False
    return True, ""

def reset(arm_move_group, req):
    if req.reset_position:
        arm_move_group.set_named_target(req.reset_position)
        arm_move_group.go()

def move(arm_move_group, pose):
    arm_move_group.set_pose_target(pose)
    #arm_move_group.set_pose_target(PoseStamped())
    return arm_move_group.go()

def graspThis(object_name):
    global ez_objects, gripper_name

    dbmp = DatabaseModelPose()
    dbmp.model_id = ez_objects[object_name]
    dbmp.confidence = 1
    dbmp.detector_name = "manual_detection"
    planning_req = GraspPlanning()
    target = GraspableObject()
    target.reference_frame_id = "1"
    target.potential_models = [dbmp]
    response = planning_srv(arm_name = gripper_name, target = target)

    return response.grasps

def nextGraspIndex(next_grasp_index, grasps):
    next_grasp_index += 1
    if next_grasp_index >= len(grasps):
        next_grasp_index = 0
    return next_grasp_index

def startPlanning(req):
    global keep_planning

    robot_commander = moveit_commander.RobotCommander()

    arm_move_group = moveit_commander.MoveGroupCommander(req.arm_move_group)

    keep_planning = True
    remaining_secs = req.secs_to_timeout
    timeout_disabled = req.secs_to_timeout <= 0
    t0 = time.clock()
    # TODO add info on service regarding a reset position
    try:
        holding_object = False
        graspit_grasps = graspThis(req.graspit_target_object)
        fixed_grasps = translateGraspIt2MoveIt(graspit_grasps)
        next_grasp_index = 0
        near_grasp_pose = PoseStamped()
        while keep_planning and (timeout_disabled or remaining_secs > 0) and not rospy.is_shutdown():
            if not timeout_disabled:
                remaining_secs -= time.clock() - t0
            try:
                if not holding_object:
                    near_grasp_pose = calcNearGraspPose(fixed_grasps[next_grasp_index])
                    # Did we successfully move to the pre-grasping position?
                    if move(arm_move_group, near_grasp_pose):
                        print "Reached pregrasp pose!"
                        if move(arm_move_group, fixed_grasps[next_grasp_index]):
                            print "Reached grasp pose!"
                            # TODO send grasp command
                            print "Holding the object!"
                            holding_object = True
                            continue
                        else:
                            # TODO don;t get stuck here!
                            pass
                    else:
                        reset(arm_move_group, req)
                        next_grasp_index = nextGraspIndex(next_grasp_index, fixed_grasps)
                else:
                    # TODO use the pregrasp position as a post grasp position too
                    pass
                #keep_planning = False
            except Exception as e:
                print str(e)
    except Exception as e:
        print str(e)
        return False, str(e)
    if not timeout_disabled and remaining_secs <= 0:
        return False, "Timeout!"
    return True, ""

# Check if the input of the scene setup service is valid
def validSceneSetupInput(req):
    tmp = dict()
    tmp2 = EzSceneSetupResponse()
    info = []
    error_codes = []
    if len(req.finger_joint_names) == 0:
        info.append("Invalid service input: No finger_joint_names provided")
        error_codes.append(tmp2.NO_FINGER_JOINTS)
        return False, info, error_codes
    if req.gripper.name == "":
        info.append("Invalid service input: No gripper name provided")
        error_codes.append(tmp2.NO_NAME)
        return False, info, error_codes
    if req.gripper.graspit_file == "":
        info.append("Invalid service input: No graspit filename provided for the gripper")
        error_codes.append(tmp2.NO_FILENAME)
        return False, info, error_codes
    if req.pose_factor <= 0:
        info.append("Invalid service input: pose_factor cannot be negative or zero")
        error_codes.append(tmp2.INVALID_POSE_FACTOR)
        return False, info, error_codes

    for obj in req.objects:
        if obj.name == "":
            info.append("Invalid service input: No object name provided")
            error_codes.append(tmp2.NO_NAME)
            return False, info, error_codes
        if obj.name in tmp:
            info.append("Invalid service input: Duplicate name: " + obj.name)
            error_codes.append(tmp2.DUPLICATE_NAME)
            return False, info, error_codes
        else:
            tmp[obj.name] = 0
        if obj.graspit_file == "" and obj.moveit_file == "":
            info.append("Invalid service input: No file provided for object: " + obj.name)
            error_codes.append(tmp2.NO_FILENAME)
            return False, info, error_codes
        if obj.pose.header.frame_id == "":
            info.append("Invalid service input: No frame_id in PoseStamped message of object: " + obj.name)
            error_codes.append(tmp2.NO_FRAME_ID)
            return False, info, error_codes

    for obs in req.obstacles:
        if obs.name == "":
            info.append("Invalid service input: No obstacle name provided")
            error_codes.append(tmp2.NO_NAME)
            return False, info, error_codes
        if obs.name in tmp:
            info.append("Invalid service input: Duplicate name: " + obs.name)
            error_codes.append(tmp2.DUPLICATE_NAME)
            return False, info, error_codes
        else:
            tmp[obs.name] = 0
        if obs.graspit_file == "" and obs.moveit_file == "":
            info.append("Invalid service input: No file provided for obstacle: " + obs.name)
            error_codes.append(tmp2.NO_FILENAME)
            return False, info, error_codes
        if obs.pose.header.frame_id == "":
            info.append("Invalid service input: No frame_id in PoseStamped message of obstacle: " + obs.name)
            error_codes.append(tmp2.NO_FRAME_ID)
            return False, info, error_codes
    return True, info, error_codes

# Graspit bodies are always referenced relatively to the "world" frame
def fixItForGraspIt(obj, pose_factor):
    global tf_listener
    p = Pose()
    if obj.pose.header.frame_id == "world":
        p.position.x = obj.pose.pose.position.x * pose_factor
        p.position.y = obj.pose.pose.position.y * pose_factor
        p.position.z = obj.pose.pose.position.z * pose_factor
        p.orientation.x = obj.pose.pose.orientation.x
        p.orientation.y = obj.pose.pose.orientation.y
        p.orientation.z = obj.pose.pose.orientation.z
        p.orientation.w = obj.pose.pose.orientation.w
        #TODO orientation?
    else:
        try:
            transform = TransformStamped()
            transform.header.stamp = rospy.Time.now()
            transform.header.frame_id = obj.pose.header.frame_id
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
        except Exception as e:
            print e

    return p

# TODO ensure that it is the y axis, just to be sure that the comment is accurate
# GraspIt and MoveIt appear to have a 90 degree difference in the y (?) axis
def translateGraspIt2MoveIt(grasps):
    # TODO
    fixed_grasps = []
    for g in grasps:
        p = PoseStamped()
        p.header.frame_id = "world"
        p.pose = g.grasp_pose.pose
        fixed_grasps.append(p)
    return fixed_grasps

def calcNearGraspPose(pose):
    # TODO
    near_pose = pose
    return near_pose

def scene_setup(req):
    global add_model_srv, load_model_srv, planning_srv, tf_listener, moveit_scene
    global ez_objects, ez_obstacles, gripper_name

    valid, info, ec = validSceneSetupInput(req)

    if not valid:
        return valid, info, ec

    res = EzSceneSetupResponse()
    res.success = True

    try:
        for obj in req.objects:
            # ------ Graspit world ------
            if obj.graspit_file != "":
                atd = AddToDatabaseRequest()
                atd.filename = obj.graspit_file
                atd.isRobot = False
                atd.asGraspable = True
                atd.modelName = obj.name
                response = add_model_srv(atd)
                if response.returnCode != response.SUCCESS:
                    res.success = False
                    res.info.append("Error adding object " + obj.name + " to graspit database")
                    res.error_codes.append(response.returnCode)
                else:
                    objectID = response.modelID
                    ez_objects[obj.name] = objectID

                    loadm = LoadDatabaseModelRequest()
                    loadm.model_id = objectID
                    loadm.model_pose = fixItForGraspIt(obj, req.pose_factor)
                    response = load_model_srv(loadm)

                    if response.result != response.LOAD_SUCCESS:
                        res.success = False
                        res.info.append("Error loading object " + obj.name + " to graspit world")
                        res.error_codes.append(response.result)
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
                if response.returnCode != response.SUCCESS:
                    res.success = False
                    res.info.append("Error adding obstacle " + obstacle.name + " to graspit database")
                    res.error_codes.append(response.returnCode)
                else:
                    obstacleID = response.modelID
                    ez_obstacles[obstacle.name] = obstacleID

                    loadm = LoadDatabaseModelRequest()
                    loadm.model_id = obstacleID
                    loadm.model_pose = fixItForGraspIt(obstacle, req.pose_factor)
                    response = load_model_srv(loadm)

                    if response.result != response.LOAD_SUCCESS:
                        res.success = False
                        res.info.append("Error loading obstacle " + obstacle.name + " to graspit world")
                        res.error_codes.append(response.result)
            # ---------------------------

            # ------ Moveit scene -------
            if obstacle.moveit_file != "":
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
        if response.returnCode != response.SUCCESS:
                res.success = False
                res.info.append("Error adding robot " + req.gripper.name + " to graspit database")
                res.error_codes.append(response.returnCode)
        else:
            gripper_name = req.gripper.name
            robotID = response.modelID

            loadm = LoadDatabaseModelRequest()
            loadm.model_id = robotID
            p = Pose()
            gripper_pos, gripper_rot = tf_listener.lookupTransform(req.gripper_frame, "world", rospy.Time(0))
            p.position.x = gripper_pos[0] * req.pose_factor
            p.position.y = gripper_pos[1] * req.pose_factor
            p.position.z = gripper_pos[2] * req.pose_factor
            # TODO orientation is not important (right?)
            loadm.model_pose = p
            response = load_model_srv(loadm)

            if response.result != response.LOAD_SUCCESS:
                res.success = False
                res.info.append("Error loading robot " + req.gripper.name + " to graspit world")
                res.error_codes.append(response.result)
        # ---------------------------

        return res

    except Exception as e:
        info.append(str(e))
        ec.append(res.EXCEPTION)
        return False, info, ec

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
