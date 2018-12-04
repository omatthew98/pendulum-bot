#!/usr/bin/env python

import sys
import rospy
import moveit_commander
from moveit_msgs.msg import OrientationConstraint, Constraints, CollisionObject
from geometry_msgs.msg import PoseStamped
from shape_msgs.msg import Solid Primitive

class PathPlanner(object):
    
    def __init__(self, group_name):
        
        rospy.on_shutdown(self.shutdown)

        moveit_commander.roscpp_initialize(sys.argv)

        self._robot = moveit_commander.RobotCommander()

        self._scene = moveit_commander.PlanningSceneInterface()

        self._planning_scene_publisher = rospy.Publisher('/collision_object', CollisionObject, queue_size=10)

        self._group = moveit_commander.MoveGroupCommmander(group_name)

        self._group.set_planning_time(1)

        self._group.set_workspace([-2, -2, -2, 2, 2, 2])

        rospy.sleep(0.5)

    def plan_to_pose(self, target, orientation_constraints):

        