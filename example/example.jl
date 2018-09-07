#!/usr/bin/env julia
#push!(LOAD_PATH, "/home/gstavrinos/jultemp")
#push!(LOAD_PATH, "/home/gstavrinos/manos_catkin_ws/src/ez_pick_and_place/src/")
using RobotOS
#using ez_pnp
include("../src/ez_pnp.jl")

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

gm = GripperMove("Gripper: Open", 60.0, 80.0, 80.0)
addMove(e, gm)

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

gm = GripperMove("Gripper: Close", 20.0, 80.0, 80.0)
addMove(e, gm)

am = ArmMove("Pick: retreat location", p)
addMove(e, am)

function move(ep::EzPnP, gm::GripperMove)
    gripper_service = ServiceProxy("schunk_pg70/set_position", set_position)
    wait_for_service("schunk_pg70/set_position")
    return gripper_service(set_positionRequest(gm.position, gm.velocity, gm.acceleration)).goal_accepted
end

start(e)