#!/usr/bin/env python
import time
import rospy
import random
import moveit_commander

from grasp_planning_graspit_msgs.srv import AddToDatabaseRequest, LoadDatabaseModelRequest
from ez_pick_and_place.srv import EzSceneSetupResponse, EzStartPlanning
from household_objects_database_msgs.msg import DatabaseModelPose
from geometry_msgs.msg import TransformStamped, PoseStamped, Pose
from manipulation_msgs.msg import GraspableObject
from manipulation_msgs.srv import GraspPlanning
from moveit_msgs.srv import GetPositionIKRequest
from std_srvs.srv import Trigger

# TODO check eef points with the service below
# and if it is successful, migrate everything to C++
# to avoid service calls
from moveit_msgs.srv import GetPositionIK

from ez_state import EZState

class EZToolSet():

    object_to_grasp = ""
    arm_move_group = None
    robot_commander = None
    arm_move_group_name = ""
    gripper_move_group_name = ""

    tf_listener = None
    moveit_scene = None
    planning_srv = None
    add_model_srv = None
    load_model_srv = None

    keep_planning = True

    ez_objects = dict()
    ez_obstacles = dict()

    gripper_name = None
    gripper_frame = None

    place_poses = []
    grasp_poses = []
    neargrasp_poses = []
    nearplace_poses = []

    grasp_plans = dict()
    place_plans = dict()
    pregrasp_plans = dict()
    preplace_plans = dict()
    postgrasp_plans = dict()
    postplace_plans = dict()

    compute_ik_srv = None

    #ez_state = EZState()

    def stopPlanning(self, req):
        self.keep_planning = False
        return True, ""

    def reset(self, req):
        if req.reset_position:
            self.arm_move_group.set_named_target(req.reset_position)
            return self.arm_move_group.go()

    def move(self, pose):
        self.arm_move_group.set_pose_target(pose)
        return self.arm_move_group.go()

    def graspThis(self, object_name):
        dbmp = DatabaseModelPose()
        dbmp.model_id = self.ez_objects[object_name][0]
        dbmp.confidence = 1
        dbmp.detector_name = "manual_detection"
        planning_req = GraspPlanning()
        target = GraspableObject()
        target.reference_frame_id = "1"
        target.potential_models = [dbmp]
        response = self.planning_srv(arm_name = self.gripper_name, target = target)

        return response.grasps

    # def attachThis(self, object_name):
    #     touch_links = self.robot_commander.get_link_names(self.gripper_move_group_name)

    def attachThis(self, object_name):
        touch_links = self.robot_commander.get_link_names(self.gripper_move_group_name)
        self.arm_move_group.attach_object(object_name, link_name=self.arm_move_group.get_end_effector_link(), touch_links=touch_links)

        # self.moveit_scene.attach_mesh(self.arm_move_group.get_end_effector_link(), name=object_name, pose=None, touch_links=touch_links)

    def detachThis(self, object_name):
        self.arm_move_group.detach_object(object_name)

    def nextGraspIndex(self, next_grasp_index):
        next_grasp_index += 1
        looped = False
        if next_grasp_index >= len(self.grasp_poses):
            next_grasp_index = 0
            looped = True
        return next_grasp_index, looped

    def planFromTo(self, start_state, target_pose):
        d = dict()
        if start_state == 0:
            self.arm_move_group.set_start_state_to_current_state()
            for tp in target_pose:
                self.arm_move_group.set_pose_target(tp)
                d[tp] = self.arm_move_group.plan()
        elif isinstance(start_state, basestring):
            for ss in start_state:
                target_dict = dict()
                for tp in target_pose:
                    # TODO
                    # target_dict[tp] = 
                    pass
                # d[ss] = target_dict

    def planCompletePnP(self, ez_state):
        self.arm_move_group.set_pose_targets(ez_state.toPoseList())
        # TODO divide the plan, attach object and continue planning
        # instead of planning the whole thing without an object
        return self.arm_move_group.plan()

    def uberPlan(self):
        # TODO create the EZStates somewhere else, so we can store them
        db = dict()
        # TODO These for loops are enormous
        # I need to estimate the better solutions first (impossible?)
        # or tone down the number of grasp poses

        print "Starting now!"

        valid_preg = self.discard(self.neargrasp_poses)
        valid_g = self.discard(self.grasp_poses)
        # TODO
        # The 3 lines below need to have the object attached
        valid_postg = self.discard(self.neargrasp_poses)
        valid_prep = self.discard(self.nearplace_poses)
        valid_p = self.discard(self.place_poses)
        # TODO
        # the postp needs to have the object placed for collision avoidance
        valid_postp = self.discard(self.nearplace_poses)

        print "valid_preg"
        print len(valid_preg)
        print "valid_g"
        print len(valid_g)
        print "valid_postg"
        print len(valid_postg)
        print "valid_prep"
        print len(valid_prep)
        print "valid_p"
        print len(valid_p)
        print "valid_postp"
        print len(valid_postp)

        print len(self.place_poses)

        plan1 = None
        plan2 = None
        critical_point = None

        for i in xrange(len(valid_preg[0])):
            for j in xrange(len(valid_g[0])):
                self.arm_move_group.set_start_state_to_current_state()
                self.arm_move_group.set_pose_targets([valid_preg[0][i].pose, valid_g[0][j].pose])
                plan1 = self.arm_move_group.plan()
                if len(plan1.joint_trajectory.points) > 0:
                    self.arm_move_group.set_start_state(valid_g[1][j])
                    self.attachThis(self.object_to_grasp)
                    for k in xrange(len(valid_postg[0])):
                        for l in xrange(len(valid_prep[0])):
                            for m in xrange(len(valid_p[0])):
                                self.arm_move_group.set_pose_targets([valid_postg[0][k].pose, valid_prep[0][l].pose, valid_p[0][m].pose])
                                plan2 = self.arm_move_group.plan()
                                print len(plan2.joint_trajectory.points)
                                if len(plan2.joint_trajectory.points) > 0:
                                    #for n in xrange(len(valid_p[0])):
                                    self.detachThis(self.object_to_grasp)
                                    critical_point = valid_g[0][j]
                                    print 'COMPLETE PLAN!!'
                                    return plan1, plan2, critical_point
                    self.detachThis(self.object_to_grasp)
        return None, None, None


        '''
                #for vpostg in valid_postg:
                    # for vprep in valid_prep:
                    #     for vp in valid_p:
                    #         for vpostp in valid_postp:
        '''


        # for g in self.grasp_poses:
        #     req.ik_request.group_name = self.arm_move_group_name
        #     req.ik_request.robot_state = k.solution#self.robot_commander.get_current_state()
        #     req.ik_request.avoid_collisions = True
        #     req.ik_request.pose_stamped = g
        #     print "------------------------"
        #     n = self.compute_ik_srv(req)
        #     print n
        #     if n.error_code.val == 1:
        #         found += 1
        #     total += 1
        #     print "------------------------"
        #     #for postg in self.neargrasp_poses:
        #     if n.error_code.val == 1:
        #         for prep in self.nearplace_poses:
        #             req.ik_request.group_name = self.arm_move_group_name
        #             req.ik_request.robot_state = n.solution#self.robot_commander.get_current_state()
        #             req.ik_request.avoid_collisions = True
        #             req.ik_request.pose_stamped = prep
        #             print "------------------------"
        #             l = self.compute_ik_srv(req)
        #             print l
        #             if l.error_code.val == 1:
        #                 found += 1
        #             total += 1
        #             print "------------------------"
        #             if l.error_code.val == 1:
        #                 for p in self.place_poses:
        #                     req.ik_request.group_name = self.arm_move_group_name
        #                     req.ik_request.robot_state = l.solution#self.robot_commander.get_current_state()
        #                     req.ik_request.avoid_collisions = True
        #                     req.ik_request.pose_stamped = p
        #                     print "------------------------"
        #                     m = self.compute_ik_srv(req)
        #                     print m
        #                     if m.error_code.val == 1:
        #                         found += 1
        #                     total += 1
        #                     print "------------------------"
        #                 #for postp in self.nearplace_poses:
                                  #cur_state = EZState(preg, g, postg, prep, p, postp)
                                  #db[cur_state] = self.planCompletePnP(cur_state)

    def discard(self, poses):
        validp = []
        validrs = []
        req = GetPositionIKRequest()
        req.ik_request.group_name = self.arm_move_group_name
        req.ik_request.robot_state = self.robot_commander.get_current_state()
        req.ik_request.avoid_collisions = True
        for p in poses:
            req.ik_request.pose_stamped = p
            k = self.compute_ik_srv(req)
            if k.error_code.val == 1:
                validp.append(p)
                validrs.append(k.solution)
        return [validp, validrs]

    def startPlanningCallback(self, req):
        # Initialize moveit stuff
        self.robot_commander = moveit_commander.RobotCommander()
        self.arm_move_group = moveit_commander.MoveGroupCommander(req.arm_move_group)
        self.arm_move_group_name = req.arm_move_group
        self.object_to_grasp = req.graspit_target_object
        self.gripper_move_group_name = req.gripper_move_group
        # Call graspit
        graspit_grasps = self.graspThis(req.graspit_target_object)
        # Generate grasp poses
        self.translateGraspIt2MoveIt(graspit_grasps, req.graspit_target_object)
        # Generate near grasp poses
        self.neargrasp_poses = self.generateNearPoses(self.grasp_poses)
        # Generate place poses
        self.place_poses = self.calcTargetPoses(self.neargrasp_poses, req.target_place)
        # Generate near place poses
        self.nearplace_poses = self.generateNearPoses(self.place_poses)

        # Generate a plan for every combination :)
        plan1, plan2, critical_point = self.uberPlan()

        if plan1 and plan2:
            step1 = self.arm_move_group.execute(plan1)
            self.attachThis(self.object_to_grasp)
            print "BEFORE"
            time.sleep(2)
            print "AFTER"
            # TODO check here for start state deviation
            self.move(critical_point)
            step2 = self.arm_move_group.execute(plan2)
            self.detachThis(self.object_to_grasp)
            if step1 and step2:
                return True, ""
            else:
                return False, "A plan was found but failed during execution..."
        else:
            return False, "No valid plan was found..."


        # # Generate plans for current to near grasp poses
        # self.pregrasp_plans = planFromTo(0, nearplace_poses)
        # # Generate plans for near grasp poses to grasp poses
        # self.grasp_plans = planFromTo()
        # # Generate plans for grasp poses to near grasp poses
        # self.postgrasp_plans = planFromTo()
        # # Generate plans for near grasp poses to near place poses
        # self.preplace_plans = planFromTo()
        # # Generate plans for near place poses to place pose
        # self.place_plans = planFromTo()
        # # Generate plans for place pose to near place poses
        # self.postplace_plans = planFromTo()



    # TODO use set_start_state of the move group
    # so that we can plan the whole thing without 
    # moving the robot!
    # TODO2 send a moveit goal for the gripper positions,
    # and since SCHUNK PG70 drivers suck,
    # create a wrapper of those messages to make the required
    # service calls to the PG70 drivers
    def startPlanning(self, req):
        self.robot_commander = moveit_commander.RobotCommander()

        self.arm_move_group = moveit_commander.MoveGroupCommander(req.arm_move_group)

        self.keep_planning = True
        remaining_secs = req.secs_to_timeout
        timeout_disabled = req.secs_to_timeout <= 0
        t0 = time.clock()
        on_reset_pose = False
        away_from_grasp_pose = True
        # TODO add info on service regarding a reset position
        try:
            holding_object = False
            graspit_grasps = self.graspThis(req.graspit_target_object)
            self.translateGraspIt2MoveIt(graspit_grasps, req.graspit_target_object)
            next_grasp_index = 0
            near_grasp_pose = PoseStamped()
            near_place_pose = PoseStamped()
            while self.keep_planning and (timeout_disabled or remaining_secs > 0) and not rospy.is_shutdown():
                if not timeout_disabled:
                    remaining_secs -= time.clock() - t0
                try:
                    if not holding_object:
                        near_grasp_pose = self.calcNearGraspPose(self.grasp_poses[next_grasp_index])
                        # Did we successfully move to the pre-grasping position?
                        if self.move(near_grasp_pose):
                            on_reset_pose = False
                            print "Reached pregrasp pose!"
                            time.sleep(2)
                            if self.move(self.grasp_poses[next_grasp_index]):
                                away_from_grasp_pose = False
                                print "Reached grasp pose!"
                                time.sleep(2)
                                # TODO send grasp command
                                print "Holding the object!"
                                self.attachThis(req.graspit_target_object)
                                time.sleep(5)
                                holding_object = True
                                continue
                            else:
                                next_grasp_index, looped = self.nextGraspIndex(next_grasp_index)
                                if looped and req.allow_replanning:
                                    graspit_grasps = self.graspThis(req.graspit_target_object)
                                    self.translateGraspIt2MoveIt(graspit_grasps, req.graspit_target_object)
                                    next_grasp_index = 0
                                continue
                        else:
                            if not on_reset_pose and self.reset(req):
                                on_reset_pose = True
                            next_grasp_index, looped = self.nextGraspIndex(next_grasp_index)
                            if looped and req.allow_replanning:
                                graspit_grasps = self.graspThis(req.graspit_target_object)
                                self.translateGraspIt2MoveIt(graspit_grasps, req.graspit_target_object)
                                next_grasp_index = 0
                    else:
                        target_pose = self.calcTargetPose(req.target_place, near_grasp_pose)
                        near_place_pose = self.calcNearPlacePose(target_pose)
                        if not away_from_grasp_pose and self.move(near_grasp_pose):
                            on_reset_pose = False
                            away_from_grasp_pose = True
                            print "Reached postgrasp pose!"
                            time.sleep(2)
                            continue
                        elif away_from_grasp_pose and self.move(near_place_pose):
                            print "Reached preplace pose!"
                            time.sleep(2)
                            if self.move(target_pose):
                                print "Reached place pose!"
                                time.sleep(2)
                                # TODO send ungrip command
                                print "Placed the object!"
                                self.moveit_scene.remove_attached_object(self.arm_move_group.get_end_effector_link(), req.graspit_target_object)
                                time.sleep(5)
                                holding_object = False
                                # stop trying now, but also try as a last move to
                                # reach the preplace pose again
                                self.move(near_place_pose)
                                return True, "That was smoooooth :)"
                        elif not on_reset_pose and self.reset(req):
                                on_reset_pose = True
                                away_from_grasp_pose = True

                except Exception as e:
                    print str(e)
        except Exception as e:
            print str(e)
            return False, str(e)
        if not timeout_disabled and remaining_secs <= 0:
            return False, "Timeout!"
        return True, ""

    # Check if the input of the scene setup service is valid
    def validSceneSetupInput(self, req):
        tmp = dict()
        tmp2 = EzSceneSetupResponse()
        info = []
        error_codes = []
        if len(req.finger_joint_names) == 0:
            info.append("Invalid service input: No finger_joint_names provided")
            error_codes.append(tmp2.NO_FINGER_JOINTS)
            return False, info, error_codes
        if req.gripper.name == "":
            info.append("Invalid service input: No gripper name provided")
            error_codes.append(tmp2.NO_NAME)
            return False, info, error_codes
        if req.gripper.graspit_file == "":
            info.append("Invalid service input: No graspit filename provided for the gripper")
            error_codes.append(tmp2.NO_FILENAME)
            return False, info, error_codes
        if req.pose_factor <= 0:
            info.append("Invalid service input: pose_factor cannot be negative or zero")
            error_codes.append(tmp2.INVALID_POSE_FACTOR)
            return False, info, error_codes

        for obj in req.objects:
            if obj.name == "":
                info.append("Invalid service input: No object name provided")
                error_codes.append(tmp2.NO_NAME)
                return False, info, error_codes
            if obj.name in tmp:
                info.append("Invalid service input: Duplicate name: " + obj.name)
                error_codes.append(tmp2.DUPLICATE_NAME)
                return False, info, error_codes
            else:
                tmp[obj.name] = 0
            if obj.graspit_file == "" and obj.moveit_file == "":
                info.append("Invalid service input: No file provided for object: " + obj.name)
                error_codes.append(tmp2.NO_FILENAME)
                return False, info, error_codes
            if obj.pose.header.frame_id == "":
                info.append("Invalid service input: No frame_id in PoseStamped message of object: " + obj.name)
                error_codes.append(tmp2.NO_FRAME_ID)
                return False, info, error_codes

        for obs in req.obstacles:
            if obs.name == "":
                info.append("Invalid service input: No obstacle name provided")
                error_codes.append(tmp2.NO_NAME)
                return False, info, error_codes
            if obs.name in tmp:
                info.append("Invalid service input: Duplicate name: " + obs.name)
                error_codes.append(tmp2.DUPLICATE_NAME)
                return False, info, error_codes
            else:
                tmp[obs.name] = 0
            if obs.graspit_file == "" and obs.moveit_file == "":
                info.append("Invalid service input: No file provided for obstacle: " + obs.name)
                error_codes.append(tmp2.NO_FILENAME)
                return False, info, error_codes
            if obs.pose.header.frame_id == "":
                info.append("Invalid service input: No frame_id in PoseStamped message of obstacle: " + obs.name)
                error_codes.append(tmp2.NO_FRAME_ID)
                return False, info, error_codes
        return True, info, error_codes

    # Graspit bodies are always referenced relatively to the "world" frame
    def fixItForGraspIt(self, obj, pose_factor):
        p = Pose()
        if obj.pose.header.frame_id == "world":
            p.position.x = obj.pose.pose.position.x * pose_factor
            p.position.y = obj.pose.pose.position.y * pose_factor
            p.position.z = obj.pose.pose.position.z * pose_factor
            p.orientation.x = obj.pose.pose.orientation.x
            p.orientation.y = obj.pose.pose.orientation.y
            p.orientation.z = obj.pose.pose.orientation.z
            p.orientation.w = obj.pose.pose.orientation.w
            #TODO orientation?
        else:
            try:
                transform = TransformStamped()
                transform.header.stamp = rospy.Time.now()
                transform.header.frame_id = obj.pose.header.frame_id
                transform.child_frame_id = "ez_fix_it_for_grasp_it"
                transform.transform.translation.x = obj.pose.pose.position.x
                transform.transform.translation.y = obj.pose.pose.position.y
                transform.transform.translation.z = obj.pose.pose.position.z
                transform.transform.rotation.x = obj.pose.pose.orientation.x
                transform.transform.rotation.y = obj.pose.pose.orientation.y
                transform.transform.rotation.z = obj.pose.pose.orientation.z
                transform.transform.rotation.w = obj.pose.pose.orientation.w
                self.tf_listener.setTransform(transform, "fixItForGraspIt")

                trans, rot = self.tf_listener.lookupTransform("ez_fix_it_for_grasp_it", "world", rospy.Time(0))

                p.position.x = trans[0] * pose_factor
                p.position.y = trans[1] * pose_factor
                p.position.z = trans[2] * pose_factor
                p.orientation.x = rot[0]
                p.orientation.y = rot[1]
                p.orientation.z = rot[2]
                p.orientation.w = rot[3]
            except Exception as e:
                print e

        return p

    # GraspIt and MoveIt appear to have a 90 degree difference in the x axis (roll 90 degrees)
    def translateGraspIt2MoveIt(self, grasps, object_name):
        self.grasp_poses = []
        for g in grasps:
            try:
                # World -> Object
                transform = TransformStamped()
                transform.header.stamp = rospy.Time.now()
                transform.header.frame_id = "world"
                transform.child_frame_id = "target_object_frame"
                transform.transform.translation.x = self.ez_objects[object_name][1].pose.position.x
                transform.transform.translation.y = self.ez_objects[object_name][1].pose.position.y
                transform.transform.translation.z = self.ez_objects[object_name][1].pose.position.z
                transform.transform.rotation.x = self.ez_objects[object_name][1].pose.orientation.x
                transform.transform.rotation.y = self.ez_objects[object_name][1].pose.orientation.y
                transform.transform.rotation.z = self.ez_objects[object_name][1].pose.orientation.z
                transform.transform.rotation.w = self.ez_objects[object_name][1].pose.orientation.w
                self.tf_listener.setTransform(transform, "ez_helper")

                # Object -> Gripper
                transform = TransformStamped()
                transform.header.stamp = rospy.Time.now()
                transform.header.frame_id = "target_object_frame"
                transform.child_frame_id = "ez_helper_graspit_pose"
                transform.transform.translation.x = g.grasp_pose.pose.position.x
                transform.transform.translation.y = g.grasp_pose.pose.position.y
                transform.transform.translation.z = g.grasp_pose.pose.position.z
                transform.transform.rotation.x = g.grasp_pose.pose.orientation.x
                transform.transform.rotation.y = g.grasp_pose.pose.orientation.y
                transform.transform.rotation.z = g.grasp_pose.pose.orientation.z
                transform.transform.rotation.w = g.grasp_pose.pose.orientation.w
                self.tf_listener.setTransform(transform, "ez_helper")

                transform_frame_gripper_trans, transform_frame_gripper_rot = self.tf_listener.lookupTransform(self.arm_move_group.get_end_effector_link(), self.gripper_frame, rospy.Time(0))

                # Gripper -> End Effector
                transform = TransformStamped()
                transform.header.stamp = rospy.Time.now()
                transform.header.frame_id = "ez_helper_graspit_pose"
                transform.child_frame_id = "ez_helper_fixed_graspit_pose"
                transform.transform.translation.x = -transform_frame_gripper_trans[0]
                transform.transform.translation.y = -transform_frame_gripper_trans[1]
                transform.transform.translation.z = -transform_frame_gripper_trans[2]
                transform.transform.rotation.x = transform_frame_gripper_rot[0]
                transform.transform.rotation.y = transform_frame_gripper_rot[1]
                transform.transform.rotation.z = transform_frame_gripper_rot[2]
                transform.transform.rotation.w = transform_frame_gripper_rot[3]
                self.tf_listener.setTransform(transform, "ez_helper")

                # Graspit to MoveIt translation
                # (Gripper -> Gripper)
                graspit_moveit_transform = TransformStamped()
                graspit_moveit_transform.header.stamp = rospy.Time.now()
                graspit_moveit_transform.header.frame_id = "ez_helper_fixed_graspit_pose"
                graspit_moveit_transform.child_frame_id = "ez_helper_target_graspit_pose"
                graspit_moveit_transform.transform.rotation.x = 0.7071
                graspit_moveit_transform.transform.rotation.y = 0.0
                graspit_moveit_transform.transform.rotation.z = 0.0
                graspit_moveit_transform.transform.rotation.w = 0.7071
                self.tf_listener.setTransform(graspit_moveit_transform, "ez_helper")

                target_trans, target_rot = self.tf_listener.lookupTransform("world", "ez_helper_target_graspit_pose", rospy.Time(0))

                g.grasp_pose.header.frame_id = "world"
                g.grasp_pose.pose.position.x = target_trans[0]
                g.grasp_pose.pose.position.y = target_trans[1]
                g.grasp_pose.pose.position.z = target_trans[2]
                g.grasp_pose.pose.orientation.x = target_rot[0]
                g.grasp_pose.pose.orientation.y = target_rot[1]
                g.grasp_pose.pose.orientation.z = target_rot[2]
                g.grasp_pose.pose.orientation.w = target_rot[3]
                self.grasp_poses.append(g.grasp_pose)
            except Exception as e:
                print e

    def calcNearGraspPose(self, pose):
        # TODO fix the near strategy
        near_pose = PoseStamped()
        near_pose.header = pose.header
        near_pose.pose.position.x = pose.pose.position.x + random.uniform(-0.05, 0.05)
        near_pose.pose.position.y = pose.pose.position.y + random.uniform(-0.05, 0.05)
        near_pose.pose.position.z = pose.pose.position.z + random.uniform(-0.05, 0.15)
        near_pose.pose.orientation = pose.pose.orientation
        return near_pose

    def calcNearPlacePose(self, target_pose):
        # TODO fix the near strategy
        near_pose = PoseStamped()
        near_pose.header = target_pose.header
        near_pose.pose.position.x = target_pose.pose.position.x + random.uniform(-0.05, 0.05)
        near_pose.pose.position.y = target_pose.pose.position.y + random.uniform(-0.05, 0.05)
        near_pose.pose.position.z = target_pose.pose.position.z + random.uniform(-0.05, 0.15)
        near_pose.pose.orientation = target_pose.pose.orientation
        return near_pose

    def calcTargetPose(self, pose, grasp_pose):
        # TODO fix the situation of an exception
        # Currently, we are doomed
        target_pose = PoseStamped()
        if pose.header.frame_id != "world":
            try:
                transform = TransformStamped()
                transform.header.stamp = rospy.Time.now()
                transform.header.frame_id = pose.header.frame_id
                transform.child_frame_id = "ez_target_pose_calculator"
                transform.transform.translation.x = pose.pose.position.x
                transform.transform.translation.y = pose.pose.position.y
                transform.transform.translation.z = pose.pose.position.z
                transform.transform.rotation.x = pose.pose.orientation.x
                transform.transform.rotation.y = pose.pose.orientation.y
                transform.transform.rotation.z = pose.pose.orientation.z
                transform.transform.rotation.w = pose.pose.orientation.w
                self.tf_listener.setTransform(transform, "calcTargetPose")

                trans, rot = self.tf_listener.lookupTransform("world", "ez_target_pose_calculator", rospy.Time(0))
                target_pose.header.stamp = rospy.Time.now()
                target_pose.header.frame_id = "world"
                target_pose.pose.position.x = trans[0]
                target_pose.pose.position.y = trans[1]
                target_pose.pose.position.z = trans[2]
            except Exception as e:
                print e
        else:
            target_pose.header = pose.header
            target_pose.pose.position.x = pose.pose.position.x
            target_pose.pose.position.y = pose.pose.position.y
            target_pose.pose.position.z = pose.pose.position.z

        target_pose.pose.orientation = grasp_pose.pose.orientation
        target_pose.pose.position.z = grasp_pose.pose.position.z + 0.01
        return target_pose

    def calcTargetPoses(self, grasp_poses, pose):
        # TODO fix the situation of an exception
        # Currently, we are doomed
        target_poses = []
        target_pose = PoseStamped()
        if pose.header.frame_id != "world":
            try:
                transform = TransformStamped()
                transform.header.stamp = rospy.Time.now()
                transform.header.frame_id = pose.header.frame_id
                transform.child_frame_id = "ez_target_pose_calculator"
                transform.transform.translation.x = pose.pose.position.x
                transform.transform.translation.y = pose.pose.position.y
                transform.transform.translation.z = pose.pose.position.z
                transform.transform.rotation.x = pose.pose.orientation.x
                transform.transform.rotation.y = pose.pose.orientation.y
                transform.transform.rotation.z = pose.pose.orientation.z
                transform.transform.rotation.w = pose.pose.orientation.w
                self.tf_listener.setTransform(transform, "calcTargetPose")

                trans, rot = self.tf_listener.lookupTransform("world", "ez_target_pose_calculator", rospy.Time(0))
                target_pose.header.stamp = rospy.Time.now()
                target_pose.header.frame_id = "world"
                target_pose.pose.position.x = trans[0]
                target_pose.pose.position.y = trans[1]
                target_pose.pose.position.z = trans[2]
            except Exception as e:
                print e
        else:
            target_pose.header = pose.header
            target_pose.pose.position.x = pose.pose.position.x
            target_pose.pose.position.y = pose.pose.position.y
            target_pose.pose.position.z = pose.pose.position.z


        for grasp_pose in grasp_poses:
            target_pose.pose.orientation = grasp_pose.pose.orientation
            target_pose.pose.position.z = grasp_pose.pose.position.z + 0.01
            target_poses.append(target_pose)
        return target_poses

    def scene_setup(self, req):
        valid, info, ec = self.validSceneSetupInput(req)

        self.gripper_frame = req.gripper_frame

        if not valid:
            return valid, info, ec

        res = EzSceneSetupResponse()
        res.success = True

        try:
            for obj in req.objects:
                # ------ Graspit world ------
                if obj.graspit_file != "":
                    atd = AddToDatabaseRequest()
                    atd.filename = obj.graspit_file
                    atd.isRobot = False
                    atd.asGraspable = True
                    atd.modelName = obj.name
                    response = self.add_model_srv(atd)
                    if response.returnCode != response.SUCCESS:
                        res.success = False
                        res.info.append("Error adding object " + obj.name + " to graspit database")
                        res.error_codes.append(response.returnCode)
                    else:
                        objectID = response.modelID

                        loadm = LoadDatabaseModelRequest()
                        loadm.model_id = objectID
                        loadm.model_pose = self.fixItForGraspIt(obj, req.pose_factor)
                        response = self.load_model_srv(loadm)

                        self.ez_objects[obj.name] = [objectID, obj.pose]

                        if response.result != response.LOAD_SUCCESS:
                            res.success = False
                            res.info.append("Error loading object " + obj.name + " to graspit world")
                            res.error_codes.append(response.result)
                # ---------------------------

                # ------ Moveit scene -------
                if obj.moveit_file != "":
                    self.moveit_scene.add_mesh(obj.name, obj.pose, obj.moveit_file)
                # ---------------------------
            for obstacle in req.obstacles:
                # ------ Graspit world ------
                if obstacle.graspit_file != "":
                    atd = AddToDatabaseRequest()
                    atd.filename = obstacle.graspit_file
                    atd.isRobot = False
                    atd.asGraspable = False
                    atd.modelName = obstacle.name
                    response = self.add_model_srv(atd)
                    if response.returnCode != response.SUCCESS:
                        res.success = False
                        res.info.append("Error adding obstacle " + obstacle.name + " to graspit database")
                        res.error_codes.append(response.returnCode)
                    else:
                        obstacleID = response.modelID

                        loadm = LoadDatabaseModelRequest()
                        loadm.model_id = obstacleID
                        loadm.model_pose = self.fixItForGraspIt(obstacle, req.pose_factor)
                        response = self.load_model_srv(loadm)

                        self.ez_obstacles[obstacle.name] = [obstacleID, obstacle.pose]

                        if response.result != response.LOAD_SUCCESS:
                            res.success = False
                            res.info.append("Error loading obstacle " + obstacle.name + " to graspit world")
                            res.error_codes.append(response.result)
                # ---------------------------

                # ------ Moveit scene -------
                if obstacle.moveit_file != "":
                    self.moveit_scene.add_mesh(obstacle.name, obstacle.pose, obstacle.moveit_file)
                # ---------------------------

            # ------ Graspit world ------
            atd = AddToDatabaseRequest()
            atd.filename = req.gripper.graspit_file
            atd.isRobot = True
            atd.asGraspable = False
            atd.modelName = req.gripper.name
            atd.jointNames = req.finger_joint_names
            response = self.add_model_srv(atd)
            if response.returnCode != response.SUCCESS:
                    res.success = False
                    res.info.append("Error adding robot " + req.gripper.name + " to graspit database")
                    res.error_codes.append(response.returnCode)
            else:
                self.gripper_name = req.gripper.name
                robotID = response.modelID

                loadm = LoadDatabaseModelRequest()
                loadm.model_id = robotID
                p = Pose()
                gripper_pos, gripper_rot = self.tf_listener.lookupTransform(self.gripper_frame, "world", rospy.Time(0))
                p.position.x = gripper_pos[0] * req.pose_factor
                p.position.y = gripper_pos[1] * req.pose_factor
                p.position.z = gripper_pos[2] * req.pose_factor
                # TODO orientation is not important (right?)
                loadm.model_pose = p
                response = self.load_model_srv(loadm)

                if response.result != response.LOAD_SUCCESS:
                    res.success = False
                    res.info.append("Error loading robot " + req.gripper.name + " to graspit world")
                    res.error_codes.append(response.result)
            # ---------------------------

            return res

        except Exception as e:
            info.append(str(e))
            ec.append(res.EXCEPTION)
            return False, info, ec

    def generateNearPoses(self, grasps):
        poses = []
        for g in grasps:
            p = self.calcNearGraspPose(g)
            while p in poses:
                p = self.calcNearGraspPose(g)
            poses.append(p)
        return poses

    # TODO
    def generateInitStates(self):
        return []

