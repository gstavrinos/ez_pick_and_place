#!/usr/bin/env python
import sys
import rospy
import moveit_commander
from geometry_msgs.msg import PoseStamped

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("scene_test")
scene = moveit_commander.PlanningSceneInterface()

rospy.sleep(2)
p = PoseStamped()
p.header.stamp = rospy.Time.now()
p.header.frame_id = "base_link"
p.pose.position.x = 1
p.pose.position.y = 1
p.pose.position.z = 1
p.pose.orientation.w = 1.0
scene.add_box("Testbox", p, (0.05, 0.05, 0.05))
