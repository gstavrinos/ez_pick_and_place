#!/usr/bin/env julia
using RobotOS
using PyCall

@pyimport rospkg

@rosimport grasp_planning_graspit_msgs.srv: AddToDatabase, LoadDatabaseModel, SaveWorld
@rosimport manipulation_msgs.srv: GraspPlanning

rostypegen()

using .grasp_planning_graspit_msgs.srv
using .manipulation_msgs.srv

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

    # Add a graspable object
    object_file = rospack[:get_path]("manos_graspit_config") * "/resources/models/objects/orange_ball.xml"
    atd = AddToDatabaseRequest()
    atd.filename = object_file
    atd.isRobot = false
    atd.asGraspable = true
    atd.modelName = "orange_ball"

    response = add_model_srv(atd)
    objectID = response.modelID

    # Add an obstacle
    obstacle_file = rospack[:get_path]("manos_graspit_config") * "/resources/models/obstacles/manos_table.xml"
    atd = AddToDatabaseRequest()
    atd.filename = obstacle_file
    atd.isRobot = false
    atd.asGraspable = false
    atd.modelName = "table"

    response = add_model_srv(atd)
    obstacleID = response.modelID

    # Add the gripper
    robot_file = rospack[:get_path]("manos_graspit_config") * "/resources/models/robots/schunk_pg70/schunk_pg70.xml"
    atd = AddToDatabaseRequest()
    atd.filename = robot_file
    atd.isRobot = true
    atd.asGraspable = false
    atd.modelName = "schunk_pg70"
    atd.jointNames = ["pg70_finger1_joint", "pg70_finger2_joint"]

    response = add_model_srv(atd)
    robotID = response.modelID

    # TODO load models on correct positions
end

main()