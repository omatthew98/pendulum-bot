#!/usr/bin/python
import sys
import rospy
import numpy as np
import moveit_commander

from moveit_msgs.msg import OrientationConstraint, Constraints, CollisionObject
from geometry_msgs.msg import PoseStamped
from baxter_interface import Limb

from geometry_msgs.msg import PointStamped
from path_planner import PathPlanner

# from controller import Controller

ready = False

class Actuator(object):

    def __init__(self, sub_topic):
        self.planner = PathPlanner("right_arm")
        self.limb = Limb("right")
        self.sub = rospy.Subscriber(sub_topic, PointStamped, self.callback)

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
# Translation: [0.607, 0.534, 0.345]
# - Rotation: in Quaternion [-0.006, 0.924, -0.080, 0.373]
#             in RPY (radian) [-2.929, 0.760, -3.044]
#             in RPY (degree) [-167.843, 43.524, -174.389]

                goal.pose.position.x = 0.683
                goal.pose.position.y = 0.048
                goal.pose.position.z = 0.246

                # goal.pose.orientation.x = 0.654
                # goal.pose.orientation.y = 0.616
                # goal.pose.orientation.z = 0.325
                # goal.pose.orientation.w = -0.294
                # goal.pose.position.x = loc.point.x
                # goal.pose.position.y = loc.point.y
                # goal.pose.position.z = loc.point.z

                goal.pose.orientation.x = 0 # -0.006
                goal.pose.orientation.y = 0 # -0.042
                goal.pose.orientation.z = -0.7 # -0.696
                goal.pose.orientation.w = 0.7 #0.717

                # .271, .653, -.271, .653

                plan = self.planner.plan_to_pose(goal, [])

                raw_input("Press <Enter> to execute plan")

                if not self.planner.execute_plan(plan):
                    raise Exception("Execution failed")
            except Exception as e:
                print e
            else:
                break


def main():

    rospy.init_node("moveit_node")
    act = Actuator("world/ball/location")
    rospy.spin()

if __name__ == "__main__":
    main()