#!/usr/bin/python
import numpy as np
import sys
import rospy
import tf
import collections

from baxter_imaging.msg import BallLoc
from scipy.integrate import solve_ivp
from geometry_msgs.msg import PointStamped, PoseStamped

from baxter_interface import Limb
import moveit_commander
from baxter_interface import gripper as robot_gripper

LOCATION_TOPIC_KINECT="~location_topic/kinect"
LOCATION_TOPIC_BASE="~location_topic/base"
TARGET_FRAME="~frame/target"

class Model:

    def __init__(self, sub_topic, pub_topic):

        self.sub_topic, self.length = sub_topic, 20
        self.pivot_x, self.pivot_y, self.pivot_z = 0, 0, 0
        self.r = 1.2 # This is length of string

        raw_input("Place the ball in the default position, press [Enter] to calibrate the system, and wait...")
        self.calibration_sub = rospy.Subscriber(self.sub_topic, PointStamped, self.calibrate)

        self.pub = rospy.Publisher(self.pub_topic, PoseStamped)
        raw_input("Raise the ball to the desired starting position, press [Enter], and hold tight while I figure out where to catch!")
        self.subscriber = rospy.Subscriber(self.sub_topic, PointStamped, self.callback)

    def calibrate(self, calib_point):

        self.pivot_x, self.pivot_y, self.pivot_z = calib_point.point.x, calib_point.point.y, calib_point.point.z
        self.calibration_sub.unregister()

        # Setting pendulum pivot
        # I think z axis is pointing down

        self.pivot_z -= self.r

        print("Done calibrating!")

    def system(self, t, y):
        g = 9.8
        theta, theta_dot, phi, phi_dot = y
        A = np.array([[0, 1, 0, 0],
                    [0, 0, 0, 0],
                    [0, 0, 0, 1],
                    [0, 0, 0, 0]])

        b = np.array([0, 
                    (phi_dot**2 * np.sin(theta) * np.cos(theta)) - (g / self.r * np.sin(theta)),
                    0,
                    -2 * theta_dot * phi_dot * np.cos(theta) / np.sin(theta)])

        return np.dot(A, y) + b

    def callback(self, data):

        # print("Model got location: ", data)
        x, y, z = data.point.x, data.point.y, data.point.z
        # self.gripper_handler(x, y, z)

        theta = np.arccos(float(self.z_pivot - z) / self.r)

        if (x == 0 and y == 0) or (x == self.x_pivot) or (y == self.y_pivot):
            phi = 0
        elif x == 0:
            if y > 0:
                phi = np.pi / 2
            else:
                phi = -1 * np.pi / 2
        elif y == 0:
            phi = 0
        else:
            phi = np.arctan(float(y - self.y_pivot) / (x - self.x_pivot))

        sol = solve_ivp(self.system, [0, 10], [theta, 0, phi, 0], max_step = 0.1)

        thetas = sol.y[0]
        phis = sol.y[2]
        angles = zip(thetas, phis)

        # points = {-self.r * np.cos(theta) : (self.r * np.sin(theta) * np.cos(phi), self.r * np.sin(theta) * np.sin(phi)) for theta, phi in zip(thetas, phis)}
        # print(points)

        x_lst = [(self.x_pivot + (self.r * np.sin(theta) * np.cos(phi))) for theta, phi in zip(thetas, phis)]
        y_lst = [(self.y_pivot + (self.r * np.sin(theta) * np.sin(phi))) for theta, phi in zip(thetas, phis)]
        z_lst = [(self.z_pivot - (self.r * np.cos(theta))) for theta in thetas]

        # keys = points.keys()

        index = int(len(x_lst) * 3/4)

        # to_go = PointStamped()
        # to_go.point.x, to_go.point.y, to_go.point.z = x_lst[index], y_lst[index], z_lst[index]

        goal = PoseStamped()
        # goal.header.frame_id = "base"

        goal.pose.position.x = x_lst[index] # 0.427
        goal.pose.position.y = y_lst[index] # -0.445
        goal.pose.position.z = z_lst[index] # 0.270

        self.angle = phis[-1]

        ### Need to find someway to get from angle -> euler angles -> quaternion
        # takes roll, pitch, yaw
        q = tf.transformations.quaternion_from_euler(0, 0, self.angle)

        # q = [0.743, 0.294, 0.560, -0.220]
        
        q = normalize(q)
        goal.pose.orientation.x = q[0] # -0.006
        goal.pose.orientation.y = q[1] # -0.042
        goal.pose.orientation.z = q[2] # -0.696
        goal.pose.orientation.w = q[3] #0.717
        
        # if self.count == 500:
        #     self.pub.publish(to_go)
        # self.count += 1

        self.pub.publish(goal)
        self.subscriber.unregister()

        print("Done computing trajectory!")

def normalize(v):
    norm = np.linalg.norm(v)
    if norm == 0:
        return v
    return v / norm

def main():
    rospy.init_node('model')
    model = Model("~location_topic/base", "ball/goal")
    rospy.spin()

if __name__=='__main__':
    main()
