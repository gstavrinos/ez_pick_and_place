#!/usr/bin/env julia
#module ez_pnp
using RobotOS
using PyCall

@pyimport moveit_commander
@pyimport rospy

@rosimport geometry_msgs.msg: PoseStamped

rostypegen()

using .geometry_msgs.msg

abstract type EzMove end
abstract type ObjectToAttach end

struct ArmMove <: EzMove
    name::String
    pose::PoseStamped
end

struct GripperMove <: EzMove
    name::String
    position::Float64
    velocity::Float64
    acceleration::Float64
    grip::Bool
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
    node_name::String
    planning_time::Float64
    planning_attempts::Int32
    time_between_arm_moves::Float64
    time_between_grips::Float64
    arm_group_name::String
    gripper_group_name::String
    default_position_name::String
    reset_on_failure::Bool
    print_status::Bool
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

    function EzPnP(nn, pt, pa, tbm, tbg, agn, ggn, dpn, rof, ps)
        moveit_commander.roscpp_initialize(ARGS)
        init_node(nn)

        return new(nn, pt, pa, tbm, tbg, agn, ggn, dpn, rof, ps, [], 
            moveit_commander.RobotCommander(),  moveit_commander.PlanningSceneInterface(), 
            moveit_commander.MoveGroupCommander(agn), [])
    end

end

function move(ep::EzPnP, am::ArmMove)
    ep.arm_move_group[:set_pose_target](am.pose)
    return ep.arm_move_group[:go]()
end

function addMove(ep::EzPnP, move::EzMove)
    push!(ep.moves, move)
end

function validMoves(ep::EzPnP)
    found_grip = false
    found_arm_moves = false
    found_ungrip_after_grip = false

    last_grip_encountered = false
    if length(ep.moves) == 0
        println("[ERROR]: No arm or gripper moves found...")
        return false
    end
    for move in ep.moves
        if move isa GripperMove
            if move.grip
                if last_grip_encountered
                    println("[ERROR]: Found two consequtive grip moves (grip == true)")
                    return false
                end
                found_grip = true
            elseif found_grip
                found_ungrip_after_grip = true
            end
            last_grip_encountered = move.grip
        elseif move isa ArmMove
            found_arm_moves = true
        end
    end
    found_warn = false
    if !found_grip
        found_warn = true
        println("[WARNING]: No grip moves found (grip == true).")
    end
    if !found_ungrip_after_grip
        found_warn = true
        println("[WARNING]: No ungrip moves found (grip == false) after gripping.")
    end
    if !found_arm_moves
        found_warn = true
        println("[WARNING]: No arm moves found.")
    end
    if found_warn
        println("[INFO]: Starting execution of the routine, despite the warnings...")
    end
    return true
end

function start(ep::EzPnP)
    if !validMoves(ep)
        println("[INFO]: Exiting...")
        exit()
    end
    ep.arm_move_group[:set_planning_time](ep.planning_time)
    ep.arm_move_group[:set_num_planning_attempts](ep.planning_attempts)

    eef_link = ep.arm_move_group[:get_end_effector_link]()

    planning_frame = ep.robot_commander[:get_planning_frame]()

    # TODO isempty is not compatible with julia-0.6.4
    if length(ep.scene_interface[:get_known_object_names]()) > 0 && length(ep.ota) == 0
        println("[INFO]: Found planning scene objects but none to attach!")
    end

    move_index = 1
    gripping = false # The gripper is currently free

    while !is_shutdown() && !isempty(ep.moves)
        try
            suc = false
            if ep.print_status
                println("[STATUS]: Attempting...: "*ep.moves[move_index].name)
            end
            suc = move(ep, ep.moves[move_index])
            if ep.moves[move_index] isa GripperMove
                sleep(ep.time_between_grips)
            elseif ep.moves[move_index] isa ArmMove
                sleep(ep.time_between_arm_moves)
            end
            if suc
                if ep.print_status
                    println("[STATUS]: Success...: "*ep.moves[move_index].name)
                end
                if ep.moves[move_index] isa GripperMove || move_index == length(ep.moves)
                    if ep.moves[move_index] isa GripperMove 
                        if ep.moves[move_index].grip
                            # attach first object here
                            if length(ep.ota) > 0
                                if ep.ota[1] isa BoxToAttach
                                    if ep.print_status
                                        println("[STATUS]: Attaching object...: "*ep.ota[1].name)
                                    end
                                    ep.scene_interface[:attach_box](ep.ota[1].link, ep.ota[1].name, ep.ota[1].pose, ep.ota[1].sz, ep.ota[1].touch_links)
                                    gripping = true
                                elseif ep.ota[1] isa MeshToAttach
                                    if ep.print_status
                                        println("[STATUS]: Attaching object...: "*ep.ota[1].name)
                                    end
                                    ep.scene_interface[:attach_mesh](ep.ota[1].link, ep.ota[1].name, ep.ota[1].pose, ep.ota[1].filename, ep.ota[1].sz, ep.ota[1].touch_links)
                                    gripping = true
                                end
                            end
                        elseif !ep.moves[move_index].grip && gripping
                            if ep.print_status
                                println("[STATUS]: Detaching object...: "*ep.ota[1].name)
                            end
                            ep.scene_interface[:remove_attached_object](ep.ota[1].link, ep.ota[1].name)
                            gripping = false
                            # TODO popfirst! is not available in julia-0.6.4
                            #popfirst!(ep.ota)
                            deleteat!(ep.ota, 1)
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
                println("[INFO]: Resetting position due to failure")
                ep.arm_move_group[:set_named_target](ep.default_position_name)
                ep.arm_move_group[:go]()
            end
        catch e
            println(e)
        end
    end
    if ep.print_status
        println("[STATUS]: Done! :)")
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
    # TODO isempty is not compatible with julia-0.6.4
    if length(touch_links) == 0
        touch_links = ep.robot_commander[:get_link_names](ep.gripper_group_name)
    end
    bta = BoxToAttach(link, name, pose, s, touch_links)
    push!(ep.ota, bta)
end

function attachMesh(ep::EzPnP, link::String, name::String, pose::PoseStamped, filename::String, s::Tuple, touch_links::Array{String})
    # TODO isempty is not compatible with julia-0.6.4
    if length(touch_links) == 0
        touch_links = ep.robot_commander[:get_link_names](ep.gripper_group_name)
    end
    mta = MeshToAttach(link, name, pose, filename, s, touch_links)
    push!(ep.ota, mta)
end

#end