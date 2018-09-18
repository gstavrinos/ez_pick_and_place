#!/usr/bin/env julia
using RobotOS
using PyCall

@pyimport rospkg
@rosimport schunk_pg70.srv: set_position
rostypegen()
using .schunk_pg70.srv

rospack = rospkg.RosPack()
ez_pnp_path = rospack[:get_path]("ez_pick_and_place") * "/src/ez_pnp.jl"
include(ez_pnp_path)

# Implement move function for gripper (ez_pnp knows how to handle this) [thanks multiple dispatch!]
function move(ep::EzPnP, gm::GripperMove)
    gripper_service = ServiceProxy("schunk_pg70/set_position", set_position)
    wait_for_service("schunk_pg70/set_position")
    return gripper_service(set_positionRequest(gm.position, gm.velocity, gm.acceleration)).goal_accepted
end

e = EzPnP("ezpnp_tester", 10.0, 10, 0, 2, "arm", "gripper", "up", true, true)

# Predefined positions for this example
pick_trans = [-0.25, -0.18, 0.075]
place_trans = [0.285, 0.253, 0.87]

# ------------------- Arm and gripper movement ------------------- 

# Pick distant location
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

gm = GripperMove("Gripper: Open", 60.0, 80.0, 80.0, false)
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

gm = GripperMove("Gripper: Close", 20.0, 80.0, 80.0, true)
addMove(e, gm)

am = ArmMove("Pick: retreat location", p)
addMove(e, am)

# Place distant location
p3 = PoseStamped()
p3.header.stamp = RobotOS.now()
p3.header.frame_id = "world"
p3.pose.position.x = place_trans[1]
p3.pose.position.y = place_trans[2]
p3.pose.position.z = place_trans[3] + 0.4
# Top-down orientation
p3.pose.orientation.x = -0.71171
p3.pose.orientation.y = -0.017323
p3.pose.orientation.z = 0.0038264
p3.pose.orientation.w = 0.70225

am = ArmMove("Place: distant location", p3)
addMove(e, am)

# Place approach location
p4 = PoseStamped()
p4.header.stamp = RobotOS.now()
p4.header.frame_id = "world"
p4.pose.position.x = place_trans[1]
p4.pose.position.y = place_trans[2]
p4.pose.position.z = place_trans[3] + 0.3
# Top-down orientation
p4.pose.orientation.x = -0.71171
p4.pose.orientation.y = -0.017323
p4.pose.orientation.z = 0.0038264
p4.pose.orientation.w = 0.70225

am = ArmMove("Place: approach location", p4)
addMove(e, am)

gm = GripperMove("Gripper: Open", 60.0, 80.0, 80.0, false)
addMove(e, gm)

am = ArmMove("Place: retreat location", p3)
addMove(e, am)


# ------------------- Scene Objects ------------------- 

# First object
ball_pose = PoseStamped()
ball_pose.header.stamp = RobotOS.now()
ball_pose.header.frame_id = "base_link"
ball_pose.pose.position.x = pick_trans[1]
ball_pose.pose.position.y = pick_trans[2]
ball_pose.pose.position.z = pick_trans[3] + 1
ball_pose.pose.orientation.x = 0
ball_pose.pose.orientation.y = 0
ball_pose.pose.orientation.z = 0
ball_pose.pose.orientation.w = 1

# Second object
box_pose = PoseStamped()
box_pose.header.stamp = RobotOS.now()
box_pose.header.frame_id = "base_link"
box_pose.pose.position.x = 1
box_pose.pose.position.y = 1
box_pose.pose.position.z = 1
box_pose.pose.orientation.x = 0
box_pose.pose.orientation.y = 0
box_pose.pose.orientation.z = 0
box_pose.pose.orientation.w = 1

attached_pose = PoseStamped()
attached_pose.header.stamp = RobotOS.now()
attached_pose.header.frame_id = "ee_link"
attached_pose.pose.position.x = 0.15

addSphere(e, "Orange Ball", ball_pose, 0.06)
addBox(e, "Testbox", box_pose, (0.05, 0.05, 0.05))
attachBox(e, "gripper_link", "Testbox", attached_pose, (0.05, 0.05, 0.05), String[])

# ------------------- Let's do it ------------------- 
start(e)