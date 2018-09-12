#!/usr/bin/env julia
#module ez_pnp
using RobotOS
using PyCall

@pyimport moveit_commander
@pyimport rospy

@rosimport geometry_msgs.msg: PoseStamped
@rosimport schunk_pg70.srv: set_position

rostypegen()

using .geometry_msgs.msg
using .schunk_pg70.srv

abstract type EzMove end
abstract type ObjectToAttach end

struct ArmMove <: EzMove
    name::String
    pose::PoseStamped
end

struct GripperMove <: EzMove
    grip::Bool
    name::String
    position::Float64
    velocity::Float64
    acceleration::Float64
end

struct BoxToAttach <: ObjectToAttach
    link::String
    name::String
    pose::PoseStamped
    sz::Tuple
    touch_links::Array{String}
end

struct MeshToAttach <: ObjectToAttach
    link::String
    name::String
    pose::PoseStamped
    filename::String
    sz::Tuple
    touch_links::Array{String}
end

struct EzPnP
    # Vars in the struct you can configure.
    default_position_name::String
    time_between_moves::Float64
    time_between_grips::Float64
    gripper_group_name::String
    planning_attempts::Int32
    planning_time::Float64
    arm_group_name::String
    reset_on_failure::Bool
    print_status::Bool
    node_name::String
    # ----------------------------------------------------------

    # This var can be modified using the addMove func.
    moves::Array{EzMove}
    # ----------------------------------------------------------

    # MoveIt-related vars. There is no need to be modified.
    robot_commander::PyObject
    scene_interface::PyObject
    arm_move_group::PyObject
    # ----------------------------------------------------------

    # This var can be modified using the addAttachedObject func.
    ota::Array{ObjectToAttach}
    # ----------------------------------------------------------

    function EzPnP(dpn, tbm, tbg, ggn, pa, pt, agn, rof, ps, nn)
        init_node(nn)
        moveit_commander.roscpp_initialize(ARGS)

        return new(dpn, tbm, tbg, ggn, pa, pt, agn, rof, ps, nn, [], moveit_commander.RobotCommander(), moveit_commander.PlanningSceneInterface(), moveit_commander.MoveGroupCommander(agn), [])
    end

end

function move(ep::EzPnP, am::ArmMove)
    ep.arm_move_group[:set_pose_target](am.pose)
    return ep.arm_move_group[:go]()
end

function addMove(ep::EzPnP, move::EzMove)
    push!(ep.moves, move)
end

function start(ep::EzPnP)

    ep.arm_move_group[:set_planning_time](ep.planning_time)
    ep.arm_move_group[:set_num_planning_attempts](ep.planning_attempts)

    eef_link = ep.arm_move_group[:get_end_effector_link]()
    println(eef_link)

    planning_frame = ep.robot_commander[:get_planning_frame]()
    println(planning_frame)

    # TODO get attached objects and known objects and print a warning
    # if you don't find anyh attached objects.

    move_index = 1
    gripping = false # The gripper is currently free

    while !is_shutdown() && !isempty(ep.moves)
        try
            suc = false
            if ep.print_status
                println("Attempting...: "*ep.moves[move_index].name)
            end
            suc = move(ep, ep.moves[move_index])
            if suc
                if ep.print_status
                    println("Success...: "*ep.moves[move_index].name)
                end
                if ep.moves[move_index] isa GripperMove || move_index == length(ep.moves)
                    if ep.moves[move_index] isa GripperMove 
                        if ep.moves[move_index].grip
                            # attach first object here
                            if length(ep.ota) > 0
                                if ep.ota[1] isa BoxToAttach
                                    if ep.print_status
                                        println("Attaching object...: "*ep.ota[1].name)
                                    end
                                    ep.scene_interface[:attach_box](ep.ota[1].link, ep.ota[1].name, ep.ota[1].pose, ep.ota[1].sz, ep.ota[1].touch_links)
                                    gripping = true
                                elseif ep.ota[1] isa MeshToAttach
                                    if ep.print_status
                                        println("Attaching object...: "*ep.ota[1].name)
                                    end
                                    ep.scene_interface[:attach_mesh](ep.ota[1].link, ep.ota[1].name, ep.ota[1].pose, ep.ota[1].filename, ep.ota[1].sz, ep.ota[1].touch_links)
                                    gripping = true
                                end
                            end
                        elseif ep.moves[move_index].grip && gripping
                            ep.scene_interface[:remove_attached_object](ep.ota[1].link, ep.ota[1].name)
                            gripping = false
                            popfirst!(ep.ota)
                        end
                    end
                    deleteat!(ep.moves, 1:move_index)
                    move_index = 1
                else
                    move_index += 1
                end
                sleep(1)
            end
            if !suc && ep.reset_on_failure
                move_index = 1
                println("Resetting position due to failure")
                ep.arm_move_group[:set_named_target](ep.default_position_name)
                ep.arm_move_group[:go]()
            end
        catch e
            println(e)
        end
    end
    if ep.print_status
        println("Done! :)")
    end

end

function addBox(ep::EzPnP, name::String, pose::PoseStamped, s::Tuple)
    ep.scene_interface[:add_box](name, pose, s)
end

function addMesh(ep::EzPnP, name::String, pose::PoseStamped, filename::String, s::Tuple)
    ep.scene_interface[:add_mesh](name, pose, filename, s)
end

function addPlane(ep::EzPnP, name::String, pose::PoseStamped, normal::Tuple, offset::Float64)
    ep.scene_interface[:add_plane](name, pose, normal, offset)
end

function addSphere(ep::EzPnP, name::String, pose::PoseStamped, radius::Float64)
    ep.scene_interface[:add_sphere](name, pose, radius)
end

function attachBox(ep::EzPnP, link::String, name::String, pose::PoseStamped, s::Tuple, touch_links::Array{String})
    bta = BoxToAttach(link, name, pose, s, touch_links)
    push!(ep.ota, bta)
    # TODO add a check for empty touch_links here, and force gripper links
    #ep.scene_interface.attach_box(link, name, pose, s, touch_links)
end

function attachMesh(ep::EzPnP, link::String, name::String, pose::PoseStamped, filename::String, s::Tuple, touch_links::Array{String})
    mta = MeshToAttach(link, name, pose, filename, s, touch_links)
    push!(ep.ota, mta)
    # TODO add a check for empty touch_links here, and force gripper links
    #ep.scene_interface.attach_mesh(link, name, pose, filename, s, touch_links)
end

#end