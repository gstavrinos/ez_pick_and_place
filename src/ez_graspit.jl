#!/usr/bin/env julia
using RobotOS
using PyCall

@pyimport rospkg

@rosimport grasp_planning_graspit_msgs.srv: AddToDatabase, LoadDatabaseModel, SaveWorld
@rosimport household_objects_database_msgs.msg: DatabaseModelPose
@rosimport manipulation_msgs.msg: GraspableObject, SceneRegion
@rosimport manipulation_msgs.srv: GraspPlanning
@rosimport geometry_msgs.msg: Pose, PoseStamped
@rosimport sensor_msgs.msg: PointCloud2

rostypegen()

using .household_objects_database_msgs.msg
using .grasp_planning_graspit_msgs.srv
using .manipulation_msgs.msg
using .manipulation_msgs.srv
using .geometry_msgs.msg
using .sensor_msgs.msg

function main()
    init_node("ez_graspit")
    add_model_srv = ServiceProxy("/graspit_add_to_database", AddToDatabase)
    wait_for_service("/graspit_add_to_database")
    load_model_srv = ServiceProxy("/graspit_load_model", LoadDatabaseModel)
    wait_for_service("/graspit_load_model")
    planning_srv = ServiceProxy("/graspit_eg_planning", GraspPlanning)
    wait_for_service("/graspit_eg_planning")
    save_world_srv = ServiceProxy("/graspit_save_world", SaveWorld)
    wait_for_service("/graspit_save_world")

    rospack = rospkg.RosPack()

    # Add an obstacle to the database
    obstacle_file = rospack[:get_path]("manos_graspit_config") * "/resources/models/obstacles/manos_table.xml"
    atd = AddToDatabaseRequest()
    atd.filename = obstacle_file
    atd.isRobot = false
    atd.asGraspable = false
    atd.modelName = "table"

    response = add_model_srv(atd)
    obstacleID = response.modelID

    # Add a graspable object to the database
    object_file = rospack[:get_path]("manos_graspit_config") * "/resources/models/objects/orange_ball.xml"
    atd = AddToDatabaseRequest()
    atd.filename = object_file
    atd.isRobot = false
    atd.asGraspable = true
    atd.modelName = "orange_ball"

    response = add_model_srv(atd)
    objectID = response.modelID

    # Add the gripper to the database
    robot_file = rospack[:get_path]("manos_graspit_config") * "/resources/models/robots/schunk_pg70/schunk_pg70_min_contacts.xml"
    atd = AddToDatabaseRequest()
    atd.filename = robot_file
    atd.isRobot = true
    atd.asGraspable = false
    atd.modelName = "schunk_pg70"
    atd.jointNames = ["pg70_finger1_joint", "pg70_finger2_joint"]

    response = add_model_srv(atd)
    robotID = response.modelID

    # Add the obstacle model from the database to the graspit planning scene
    loadm = LoadDatabaseModelRequest()
    loadm.model_id = obstacleID
    p = Pose()
    p.position.z = 430 # 43cm above the world link
    loadm.model_pose = p

    response = load_model_srv(loadm)

    # Add the graspable model from the database to the planning scene
    loadm = LoadDatabaseModelRequest()
    loadm.model_id = objectID
    p = Pose()
    p.position.x = 285
    p.position.y = 253
    p.position.z = 900 #86cm the height of the table, and 3cm the radius of the ball +some more
    loadm.model_pose = p

    response = load_model_srv(loadm)

    # Add the robot model from the database to the planning scene
    loadm = LoadDatabaseModelRequest()
    loadm.model_id = robotID
    p = Pose()
    # TODO listen for a TF here!
    p.position.x = 40
    p.position.y = 40
    p.position.z = 1000 + 430# 43cm above the world link
    loadm.model_pose = p

    response = load_model_srv(loadm)


    # savew = SaveWorldRequest()
    # savew.filename = "ez_graspit_exported_world.xml"
    # save_world_srv(savew)


    # So much bureaucracy and book-keeping for the planning request! Whew!
    dbmp = DatabaseModelPose()
    dbmp.model_id = objectID
    #dbmp.model_pose = 
    dbmp.confidence = 1
    dbmp.detector_name = "manual_detection"
    # go = GraspableObject()
    # go.reference_frame_id = "0" #0 = object pose not used, 1 = object pose used
    # go.potential_models = [dbmp]
    # go.region.cloud.data = []
    planning_req = GraspPlanningRequest()
    planning_req.arm_name = "arm"
    planning_req.target.reference_frame_id = "0"
    planning_req.target.potential_models = [dbmp]
    planning_req.target = GraspableObject()
    planning_req.target.region = SceneRegion()
    planning_req.target.region.cloud = PointCloud2()
    println(typeof(planning_req.target.region.cloud.data))

    t = convert(PyObject, planning_req)
    println(typeof(t[:target][:region][:cloud][:data]))
    t2 = pycall(planning_srv.o, PyObject, t)
    println(t2)

    #planning_req.collision_object_name = "??"
    #planning_req.collision_support_surface = "??"
    #planning_req.grasps_to_evaluate = []
    #planning_req.moveable_obstacles = []
    # response = planning_srv(planning_req)

    # println(response.grasps)

end

main()