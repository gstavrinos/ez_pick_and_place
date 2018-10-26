#!/usr/bin/env julia
using RobotOS
using PyCall

@pyimport moveit_commander
@pyimport rospy

@rosimport geometry_msgs.msg: PoseStamped
@rosimport ez_pick_and_place.srv: EzSceneSetup
@rosimport grasp_planning_graspit_msgs.srv: AddToDatabase, LoadDatabaseModel
@rosimport manipulation_msgs.srv: GraspPlanning

rostypegen()

using .geometry_msgs.msg
using .ez_pick_and_place.srv
using .grasp_planning_graspit_msgs.srv

planning_srv = Nothing
add_model_srv = Nothing
load_model_srv = Nothing

ez_objects = Dict()
ez_obstacles = Dict()

function validInput(req::EzSceneSetupRequest)
    return true
end

function scene_setup(req::EzSceneSetupRequest)
    res = EzSceneSetupResponse()
    if !validInput(req)
        res.success = false
        res.info = "Invalid input data"
        return res
    end
    for object in req.objects
        atd = AddToDatabaseRequest()
        atd.filename =object.file
        atd.isRobot = false
        atd.asGraspable = true
        atd.modelName = object.name
        response = add_model_srv(atd)
        objectID = response.modelID
        ez_objects[object.name] = objectID
    end
    for obstacle in req.obstacles
        atd = AddToDatabaseRequest()
        atd.filename = obstacle.file
        atd.isRobot = false
        atd.asGraspable = false
        atd.modelName = obstacle.name
        response = add_model_srv(atd)
        obstacleID = response.modelID
        ez_obstacles[obstacle.name] = obstacleID
    end
    return EzSceneSetupResponse()
end

function main()
    init_node("ez_pnp")
    scene_srv = Service("ez_pnp/scene_setup", EzSceneSetup, scene_setup)

    add_model_srv = ServiceProxy("/graspit_add_to_database", AddToDatabase)
    wait_for_service("/graspit_add_to_database")
    load_model_srv = ServiceProxy("/graspit_load_model", LoadDatabaseModel)
    wait_for_service("/graspit_load_model")
    planning_srv = ServiceProxy("/graspit_eg_planning", GraspPlanning)
    wait_for_service("/graspit_eg_planning")

    spin()
end

main()