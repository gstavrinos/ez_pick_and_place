#!/usr/bin/env julia
using RobotOS
using PyCall

@pyimport rospkg

@rosimport ez_pick_and_place.srv: EzSceneSetup
@rosimport ez_pick_and_place.msg: EzModel
@rosimport geometry_msgs.msg: PoseStamped

rostypegen()

using .ez_pick_and_place.srv
using .ez_pick_and_place.msg
using .geometry_msgs.msg

function main()
    init_node("ez_graspit")
    scene_setup_srv = ServiceProxy("/ez_pnp/scene_setup", EzSceneSetup)
    println("Waiting for the service to come up...")
    wait_for_service("/ez_pnp/scene_setup")
    println("Done!")

    rospack = rospkg.RosPack()

    req = EzSceneSetupRequest()
    req.gripper.name = "schunk_pg70"
    req.gripper.file = rospack[:get_path]("manos_graspit_config") * "/resources/models/robots/schunk_pg70/schunk_pg70_min_contacts.xml"
    req.finger_joint_names = ["pg70_finger1_joint", "pg70_finger2_joint"]
    req.pose_factor = 1000

    obstacle = EzModel()
    obstacle.file = rospack[:get_path]("manos_graspit_config") * "/resources/models/obstacles/manos_table.xml"
    obstacle.name = "table"
    obstacle.pose.pose.position.z = 0.43

    push!(req.obstacles, obstacle)

    object = EzModel()
    object.file = rospack[:get_path]("manos_graspit_config") * "/resources/models/objects/blue_pole_small.xml"
    object.name = "blue_pole_small"
    object.pose.pose.position.x = 0.285
    object.pose.pose.position.y = 0.253
    object.pose.pose.position.z = 0.96
    object.pose.pose.orientation.x = 0.5
    object.pose.pose.orientation.y = 0.5
    object.pose.pose.orientation.z = 0.5
    object.pose.pose.orientation.w = 0.5

    push!(req.objects, object)

    response = scene_setup_srv(req)

    println(response)

end

main()