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

struct ArmMove <: EzMove
    name::String
    pose::PoseStamped

    function ArmMove(n, p)
        return new(n, p)
    end
end

struct GripperMove <: EzMove
    name::String
    position::Float64
    velocity::Float64
    acceleration::Float64
end

struct EzPnP
    # Vars in the struct you can configure.
    default_position_name::String
    time_between_moves::Float64
    time_between_grips::Float64
    planning_attempts::Int32
    planning_time::Float64
    arm_group_name::String
    reset_on_failure::Bool
    print_status::Bool
    node_name::String
    # -------------------------------------

    # This var can be modified using the addMove func.
    moves::Array{EzMove}
    # ------------------------------------------------

    # MoveIt-related vars. There is no need to be modified.
    robot_commander::PyObject
    scene_interface::PyObject
    arm_move_group::PyObject
    # -----------------------------------------------------

    function EzPnP(dpn, tbm, tbg, pa, pt, agn, rof, ps, nn)
        init_node(nn)
        moveit_commander.roscpp_initialize(ARGS)

        return new(dpn, tbm, tbg, pa, pt, agn, rof, ps, nn, [], moveit_commander.RobotCommander(), moveit_commander.PlanningSceneInterface(), moveit_commander.MoveGroupCommander(agn))
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

    move_index = 1

    while !is_shutdown() && !isempty(ep.moves)
        try
            suc = false
            # TODO implement move(ep::EzPnP, gm::GripperMove)
            if ep.print_status
                println("Attempting...: "*ep.moves[move_index].name)
            end
            suc = move(ep, ep.moves[move_index])
            if suc
                if ep.print_status
                    println("Success...: "*ep.moves[move_index].name)
                end
                if ep.moves[move_index] isa GripperMove || move_index == length(ep.moves)
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
#end