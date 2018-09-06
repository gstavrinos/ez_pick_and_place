#!/usr/bin/env julia
#module EzPnP
#push!(LOAD_PATH, "/home/gstavrinos/jultemp")
#push!(LOAD_PATH, "/home/gstavrinos")
using RobotOS
using PyCall

@pyimport moveit_commander
@pyimport rospy

@rosimport geometry_msgs.msg: PoseStamped
#@rosimport schunk_pg70.srv: set_position

rostypegen()

using .geometry_msgs.msg
#using .schunk_pg70.srv

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

# function move(m::EzMove)
#     m.move()
# end

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
    ep.arm_move_group[:set_pose_target](ep.moves[1].pose)
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

    #gripper_service = ServiceProxy("schunk_pg70/set_position", set_position)
    #wait_for_service("schunk_pg70/set_position")
    #gripper_service(set_positionRequest(60, 80, 80))
    while ! is_shutdown() && !isempty(ep.moves)
        try
            suc = false
            # TODO implement move(ep::EzPnP, gm::GripperMove)
            suc = move(ep, ep.moves[1])
            if suc
                println("HELL YEAH!")
                popfirst!(ep.moves)
                #sleep(1)
            end
            if !suc && ep.reset_on_failure
                println("Resetting position due to failure")
                ep.arm_move_group[:set_named_target](ep.default_position_name)
                ep.arm_move_group[:go]()
            end
        catch e
            println("***************")
            println(e)
            println("***************")
        end
    end

end

# #################################### TEST ####################################
e = EzPnP("up", 1.0, 1.0, 10, 10.0, "arm", true, true, "ezpnp_tester")

# Pick distant location
pick_trans = [-0.25, -0.18, 0.075]
p = PoseStamped()
p.header.stamp = RobotOS.now()
p.header.frame_id = "base_link"
p.pose.position.x = pick_trans[1]
p.pose.position.y = pick_trans[2]
p.pose.position.z = pick_trans[3] + 0.4
# Top-down orientation
p.pose.orientation.x = -0.71171
p.pose.orientation.y = -0.017323
p.pose.orientation.z = 0.0038264
p.pose.orientation.w = 0.70225

am = ArmMove("Pick: distant location", p)
addMove(e, am)

# Pick approach
p2 = PoseStamped()
p2.header.stamp = RobotOS.now()
p2.header.frame_id = "base_link"
p2.pose.position.x = pick_trans[1]
p2.pose.position.y = pick_trans[2]
p2.pose.position.z = pick_trans[3] + 0.25
# Top-down orientation
p2.pose.orientation.x = -0.71171
p2.pose.orientation.y = -0.017323
p2.pose.orientation.z = 0.0038264
p2.pose.orientation.w = 0.70225

am = ArmMove("Pick: approach location", p2)
addMove(e, am)

am = ArmMove("Pick: retreat location", p)
addMove(e, am)

start(e)
#end
