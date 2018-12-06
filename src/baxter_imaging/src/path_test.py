#!/usr/bin/python
import sys
import rospy
import numpy as np
import moveit_commander

from moveit_msgs.msg import OrientationConstraint, Constraints, CollisionObject
from geometry_msgs.msg import PoseStamped
from baxter_interface import Limb

from baxter_imaging.msg import BallLoc
from path_planner import PathPlanner

# from controller import Controller

ready = False

class Actuator(object):

    def __init__(self, sub_topic):
        self.sub = rospy.Subscriber(sub_topic, BallLoc, self.callback)
        self.planner = PathPlanner("right_arm")
        self.limb = Limb("right")

        # self.orien_const = OrientationConstraint()
        # self.orien_const.link_name = "right_gripper"
        # self.orien_const.header.frame_id = "base"
        # self.orien_const.orientation.x = 1
        # self.orien_const.absolute_x_axis_tolerance = 0.1
        # self.orien_const.absolute_y_axis_tolerance = 0.1
        # self.orien_const.absolute_z_axis_tolerance = 0.1
        # self.orien_const.weight = 1.0


        # size = [1, 2, 2]
        # obstacle_pose = PoseStamped()
        # obstacle_pose.header.frame_id = "base"

        # obstacle_pose.pose.position.x = -0.7
        # obstacle_pose.pose.position.y = 0
        # obstacle_pose.pose.position.z = 0

        # obstacle_pose.pose.orientation.x = 0
        # obstacle_pose.pose.orientation.y = 0
        # obstacle_pose.pose.orientation.z = 0
        # obstacle_pose.pose.orientation.w = 1

        # self.planner.add_box_obstacle(size, "wall", obstacle_pose)

        # ready = True


    def callback(self, loc):

        # if not ready:
            # return
        
        print("Ready!")
        while not rospy.is_shutdown():
            try:
                goal = PoseStamped()
                goal.header.frame_id = "base"

                goal.pose.position.x = .75
                goal.pose.position.y = -.1
                goal.pose.position.z = .08

                goal.pose.orientation.x = 0
                goal.pose.orientation.y = -1
                goal.pose.orientation.z = 0
                goal.pose.orientation.w = 0

                # .271, .653, -.271, .653

                plan = self.planner.plan_to_pose(goal, [self.orien_const])

                raw_input("Press <Enter> to execute plan")

                if not self.planner.execute_plan(plan):
                    raise Exception("Execution failed")
            except Exception as e:
                print e
            else:
                break


def main():

    rospy.init_node("moveit_node")
    act = Actuator("kinect/ball/goal")
    rospy.spin()

if __name__ == "__main__":
    main()