#!/usr/bin/python
import sys
import rospy
import numpy as np
import moveit_commander

from moveit_msgs.msg import OrientationConstraint, Constraints, CollisionObject
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from baxter_interface import Limb
from baxter_interface import gripper as robot_gripper

from geometry_msgs.msg import PointStamped
from path_planner import PathPlanner

# from controller import Controller

ready = False

class Actuator(object):

    def __init__(self, sub_topic):

        self.planner = None
        self.limb = None
        self.executed = False

        self.planner = PathPlanner("right_arm")
        self.limb = Limb("right")

        # self.orien_const = OrientationConstraint()
        # self.orien_const.link_name = "right_gripper"
        # self.orien_const.header.frame_id = "base"
        # self.orien_const.orientation.z = -1
        # self.orien_const.absolute_x_axis_tolerance = 0.1
        # self.orien_const.absolute_y_axis_tolerance = 0.1
        # self.orien_const.absolute_z_axis_tolerance = 0.1
        # self.orien_const.weight = 1.0


        size = [1, 2, 2]
        obstacle_pose = PoseStamped()
        obstacle_pose.header.frame_id = "base"

        obstacle_pose.pose.position.x = -0.7
        obstacle_pose.pose.position.y = 0
        obstacle_pose.pose.position.z = 0

        obstacle_pose.pose.orientation.x = 0
        obstacle_pose.pose.orientation.y = 0
        obstacle_pose.pose.orientation.z = 0
        obstacle_pose.pose.orientation.w = 1

        self.planner.add_box_obstacle(size, "wall", obstacle_pose)

        self.pub = rospy.Publisher(pub_topic, Bool)
        self.sub = rospy.Subscriber(sub_topic, PoseStamped, self.callback)


    def callback(self, goal):

        while not self.executed:

            # if goal.pose.position.y <= 0:
            #     self.planner = PathPlanner("right_arm")
            #     self.limb = Limb("right")
            # else:
            #     self.planner = PathPlanner("left_arm")
            #     self.limb = Limb("left")

            try:
                plan = self.planner.plan_to_pose(goal, [])

                raw_input("Press <Enter> to execute plan")

                if not self.planner.execute_plan(plan):
                    raise Exception("Execution failed")
            except Exception as e:
                print e
            else:
                self.executed = True
                print("Execution succeeded! Release the ball when ready!")
                break

        self.pub.publish(True)


def main():

    rospy.init_node("moveit_node")
    act = Actuator("ball/goal", "ball/ready")
    rospy.spin()

if __name__ == "__main__":
    main()
