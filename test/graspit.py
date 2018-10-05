#!/usr/bin/env python
import rospy
import rospkg
from grasp_planning_graspit_msgs.srv import AddToDatabase, LoadDatabaseModel, SaveWorld
from household_objects_database_msgs.msg import DatabaseModelPose
from manipulation_msgs.msg import GraspableObject, SceneRegion
from manipulation_msgs.srv import GraspPlanning
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import PointCloud2

def main():
    rospy.init_node("ez_graspit")
    add_model_srv = rospy.ServiceProxy("/graspit_add_to_database", AddToDatabase)
    rospy.wait_for_service("/graspit_add_to_database")
    load_model_srv = rospy.ServiceProxy("/graspit_load_model", LoadDatabaseModel)
    rospy.wait_for_service("/graspit_load_model")
    planning_srv = rospy.ServiceProxy("/graspit_eg_planning", GraspPlanning)
    rospy.wait_for_service("/graspit_eg_planning")
    save_world_srv = rospy.ServiceProxy("/graspit_save_world", SaveWorld)
    rospy.wait_for_service("/graspit_save_world")

    rospack = rospkg.RosPack()

    # Add an obstacle to the database
    obstacle_file = rospack.get_path("manos_graspit_config") + "/resources/models/obstacles/manos_table.xml"
    response = add_model_srv(filename=obstacle_file, isRobot = False, asGraspable = False, modelName = "table")
    obstacleID = response.modelID

    # Add a graspable object to the database
    object_file = rospack.get_path("manos_graspit_config") + "/resources/models/objects/orange_ball.xml"
    response = add_model_srv(filename=object_file, isRobot = False, asGraspable = True, modelName = "orange_ball")
    objectID = response.modelID

    # Add the gripper to the database
    robot_file = rospack.get_path("manos_graspit_config") + "/resources/models/robots/schunk_pg70/schunk_pg70_min_contacts.xml"
    response = add_model_srv(filename=robot_file, isRobot = True, asGraspable = False, modelName = "schunk_pg70", jointNames = ["pg70_finger1_joint", "pg70_finger2_joint"])
    robotID = response.modelID

    # Add the obstacle model from the database to the graspit planning scene
    p = Pose()
    p.position.z = 430 # 43cm above the world link
    response = load_model_srv(model_id = obstacleID, model_pose = p)

    # Add the graspable model from the database to the planning scene
    p = Pose()
    p.position.x = 285
    p.position.y = 253
    p.position.z = 900 #86cm the height of the table, and 3cm the radius of the ball +some more
    response = load_model_srv(model_id = objectID, model_pose = p)

    # Add the robot model from the database to the planning scene
    p = Pose()
    # TODO listen for a TF here!
    p.position.x = 40
    p.position.y = 40
    p.position.z = 1000 + 430# 43cm above the world link
    response = load_model_srv(model_id = robotID, model_pose = p)


    # So much bureaucracy and book-keeping for the planning request! Whew!
    dbmp = DatabaseModelPose()
    dbmp.model_id = objectID
    dbmp.confidence = 1
    dbmp.detector_name = "manual_detection"
    planning_req = GraspPlanning()
    target = GraspableObject()
    target.reference_frame_id = "1"
    target.potential_models = [dbmp]
    response = planning_srv(arm_name = "schunk_pg70", target = target)

    print response.grasps

main()