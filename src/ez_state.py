#!/usr/bin/env python
class EZState():
    pregrasp_pose = None
    grasp_pose = None
    postgrasp_pose = None
    preplace_pose = None
    place_pose = None
    postplace_pose = None

    def __init__(self, preg, g, postg, prep, p, postp):
        self.pregrasp_pose = preg
        self.grasp_pose = g
        self.postgrasp_pose = postg
        self.preplace_pose = prep
        self.place_pose = p
        self.postplace_pose = postp

    def toList(self):
        return [self.pregrasp_pose, self.grasp_pose, self.postgrasp_pose, self.preplace_pose, self.place_pose, self.postplace_pose]

    def toPoseList(self):
        return [self.pregrasp_pose.pose, self.grasp_pose.pose, self.postgrasp_pose.pose, self.preplace_pose.pose, self.place_pose.pose, self.postplace_pose.pose]

    def energy(self, indeces):
        return 0