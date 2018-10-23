#!/usr/bin/env python
import tf
import rospy
from manipulation_msgs.srv import GraspPlanning
from manipulation_msgs.msg import GraspableObject
from geometry_msgs.msg import PoseStamped, TransformStamped
from household_objects_database_msgs.msg import DatabaseModelPose
from ez_pick_and_place.srv import GraspitPlanning, GraspitPlanningResponse

# INFO:
# This node is used because RobotOS.jl (currently) crashes
# when the original graspit_eg_planning service is called

original_planning_srv = None
tf_listener = None

def planIt(req):
    global original_planning_srv, tf_listener

    dbmp = DatabaseModelPose()
    dbmp.model_id = req.objectID
    dbmp.confidence = req.confidence
    dbmp.detector_name = req.detector_name
    target = GraspableObject()
    target.reference_frame_id = req.reference_frame_id
    target.potential_models = [dbmp]
    response = original_planning_srv(arm_name = req.gripper_name, target = target)
    cg = response.grasps
    cp = []
    for i in range(len(cg)):
        try:
            if len(req.transform_to_frame_id) > 0:
                transform = TransformStamped()
                transform.header.stamp = rospy.Time.now()
                transform.header.frame_id = req.object_frame
                transform.child_frame_id = "ez_helper_graspit_pose"
                transform.transform.translation.x = cg[i].grasp_pose.pose.position.x
                transform.transform.translation.y = cg[i].grasp_pose.pose.position.y
                transform.transform.translation.z = cg[i].grasp_pose.pose.position.z
                transform.transform.rotation.x = cg[i].grasp_pose.pose.orientation.x
                transform.transform.rotation.y = cg[i].grasp_pose.pose.orientation.y
                transform.transform.rotation.z = cg[i].grasp_pose.pose.orientation.z
                transform.transform.rotation.w = cg[i].grasp_pose.pose.orientation.w
                tf_listener.setTransform(transform, "ez_helper")

                graspit_moveit_transform = TransformStamped()
                graspit_moveit_transform.header.stamp = rospy.Time.now()
                graspit_moveit_transform.header.frame_id = "ez_helper_graspit_pose"
                graspit_moveit_transform.child_frame_id = "ez_helper_fixed_graspit_pose"
                graspit_moveit_transform.transform.rotation.x = 0.5
                graspit_moveit_transform.transform.rotation.y = 0.5
                graspit_moveit_transform.transform.rotation.z = 0.5
                graspit_moveit_transform.transform.rotation.w = 0.5
                tf_listener.setTransform(graspit_moveit_transform, "ez_helper")

                transform_frame_gripper_trans, transform_frame_gripper_rot = tf_listener.lookupTransform(req.gripper_frame, req.transform_to_frame_id, rospy.Time(0))

                transform = TransformStamped()
                transform.header.stamp = rospy.Time.now()
                transform.header.frame_id = "ez_helper_fixed_graspit_pose"
                transform.child_frame_id = "ez_helper_target_graspit_pose"
                transform.transform.translation.x = transform_frame_gripper_trans[0]
                transform.transform.translation.y = transform_frame_gripper_trans[1]
                transform.transform.translation.z = transform_frame_gripper_trans[2]
                transform.transform.rotation.x = transform_frame_gripper_rot[0]
                transform.transform.rotation.y = transform_frame_gripper_rot[1]
                transform.transform.rotation.z = transform_frame_gripper_rot[2]
                transform.transform.rotation.w = transform_frame_gripper_rot[3]
                tf_listener.setTransform(transform, "ez_helper")

                target_trans, target_rot = tf_listener.lookupTransform("world", "ez_helper_target_graspit_pose", rospy.Time(0))

                cg[i].grasp_pose.header.frame_id = "world"
                cg[i].grasp_pose.pose.position.x = target_trans[0]
                cg[i].grasp_pose.pose.position.y = target_trans[1]
                cg[i].grasp_pose.pose.position.z = target_trans[2]
                cg[i].grasp_pose.pose.orientation.x = target_rot[0]
                cg[i].grasp_pose.pose.orientation.y = target_rot[1]
                cg[i].grasp_pose.pose.orientation.z = target_rot[2]
                cg[i].grasp_pose.pose.orientation.w = target_rot[3]
                cp.append(cg[i].grasp_pose)
        except Exception as e:
            print e
    return GraspitPlanningResponse(cp)

def main():
    global original_planning_srv, tf_listener
    rospy.init_node("ez_helper")

    planning_srv = rospy.Service("ez_helper/graspit_eg_planning", GraspitPlanning, planIt)
    original_planning_srv = rospy.ServiceProxy("graspit_eg_planning", GraspPlanning)

    tf_listener = tf.TransformListener()

    rospy.spin()

main()