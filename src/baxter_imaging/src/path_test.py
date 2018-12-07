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

        self.orien_const = OrientationConstraint()
        self.orien_const.link_name = "right_gripper"
        self.orien_const.header.frame_id = "base"
        self.orien_const.orientation.z = -1
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

        while not rospy.is_shutdown():
            try:
                goal = PoseStamped()
                goal.header.frame_id = "base"

                goal.pose.position.x = 0.427 #loc.point.x
                goal.pose.position.y = -0.445 #loc.point.y
                goal.pose.position.z = 0.270 #loc.point.z

                q = [0.743, 0.294, 0.560, -0.220]
                
                q = normalize(q)
                goal.pose.orientation.x = q[0] # -0.006
                goal.pose.orientation.y = q[1] # -0.042
                goal.pose.orientation.z = q[2] # -0.696
                goal.pose.orientation.w = q[3] #0.717

               # .271, .653, -.271, .653

                plan = self.planner.plan_to_pose(goal, [])

                raw_input("Press <Enter> to execute plan")

                if not self.planner.execute_plan(plan):
                    raise Exception("Execution failed")
            except Exception as e:
                print e
            else:
                break

def normalize(v):
                    norm = np.linalg.norm(v)
                    if norm == 0:
                        return v
                    return v / norm


def main():

    rospy.init_node("moveit_node")
    act = Actuator("kinect/ball/goal")
    rospy.spin()

if __name__ == "__main__":
    main()