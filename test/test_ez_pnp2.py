#!/usr/bin/env python
import tf
import rospy

import rospkg

from ez_pick_and_place.srv import EzSceneSetup, EzSceneSetupRequest, EzSceneSetupResponse
from ez_pick_and_place.msg import EzModel
from geometry_msgs.msg import PoseStamped

def main():
    rospy.init_node("ez_graspit")
    scene_setup_srv = rospy.ServiceProxy("/ez_pnp/scene_setup", EzSceneSetup)
    print "Waiting for the service to come up..."
    rospy.wait_for_service("/ez_pnp/scene_setup")
    print "Done!"

    rospack = rospkg.RosPack()

    req = EzSceneSetupRequest()
    req.gripper.name = "schunk_pg70"
    req.gripper.graspit_file = rospack.get_path("manos_graspit_config") + "/resources/models/robots/schunk_pg70/schunk_pg70_min_contacts.xml"
    req.finger_joint_names = ["pg70_finger1_joint", "pg70_finger2_joint"]
    req.pose_factor = 1000

    obstacle = EzModel()
    obstacle.graspit_file = rospack.get_path("manos_graspit_config") + "/resources/models/obstacles/manos_table.xml"
    obstacle.name = "table"
    obstacle.pose.header.frame_id = "world"
    obstacle.pose.pose.position.z = 0.43

    req.obstacles.append(obstacle)

    object = EzModel()
    object.graspit_file = rospack.get_path("manos_graspit_config") + "/resources/models/objects/blue_pole_small.xml"
    object.name = "blue_pole_small"
    object.pose.header.frame_id = "world"
    object.pose.pose.position.x = 0.285
    object.pose.pose.position.y = 0.253
    object.pose.pose.position.z = 0.96
    object.pose.pose.orientation.x = 0.5
    object.pose.pose.orientation.y = 0.5
    object.pose.pose.orientation.z = 0.5
    object.pose.pose.orientation.w = 0.5

    req.objects.append(object)

    response = scene_setup_srv(req)

    print response

main()