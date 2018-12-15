#!/usr/bin/python
import numpy as np
import matplotlib.pyplot as plt
from pylab import *
from mpl_toolkits.mplot3d import Axes3D
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
        self.r = 1.08 # This is length of string

        raw_input("Place the ball in the default position, press [Enter] to calibrate the system, and wait...\n\n")
        self.calibration_sub = rospy.Subscriber(self.sub_topic, PointStamped, self.calibrate)

        self.pub = rospy.Publisher(pub_topic, PoseStamped)
        self.goal = None
        self.queue_x = []
        self.queue_y = []
        self.queue_z = []
        raw_input("Raise the ball to the desired starting position, press [Enter], and hold tight while I figure out where to catch!")
        self.subscriber = rospy.Subscriber(self.sub_topic, PointStamped, self.callback)

        self.rate = rospy.Rate(100)

    def calibrate(self, calib_point):

        self.pivot_x, self.pivot_y, self.pivot_z = calib_point.point.x, calib_point.point.y, calib_point.point.z
        self.calibration_sub.unregister()

        # Setting pendulum pivot
        # I think z axis is pointing down

        self.pivot_z += self.r

        print("Done calibrating!")
        print("Pivot at location: ", self.pivot_x, self.pivot_y, self.pivot_z)

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

        x, y, z = data.point.x, data.point.y, data.point.z
        print("Ball at location: ", x, y, z)

        if len(self.queue_x) < 50:
            self.queue_x.append(x)
            self.queue_y.append(y)
            self.queue_z.append(z)
            return

        x, y, z = [sum(queue) / len(queue) for queue in [self.queue_x, self.queue_y, self.queue_z]]

        theta = np.arccos(float(self.pivot_z - z) / self.r)

        if (x == 0 and y == 0) or (x == self.pivot_x) or (y == self.pivot_y):
            phi = 0
        elif x == 0:
            if y > 0:
                phi = np.pi / 2
            else:
                phi = -1 * np.pi / 2
        elif y == 0:
            phi = 0
        else:
            phi = np.arctan(float(y - self.pivot_y) / (x - self.pivot_x))

        sol = solve_ivp(self.system, [0, 10], [theta, 0, phi, 0], max_step = 0.01)

        thetas = sol.y[0]
        phis = sol.y[2]
        angles = zip(thetas, phis)

        # points = {-self.r * np.cos(theta) : (self.r * np.sin(theta) * np.cos(phi), self.r * np.sin(theta) * np.sin(phi)) for theta, phi in zip(thetas, phis)}
        # print(points)

        x_lst = [self.pivot_x + (self.r * np.sin(theta) * np.cos(phi)) for theta, phi in zip(thetas, phis)]
        y_lst = [self.pivot_y + (self.r * np.sin(theta) * np.sin(phi)) for theta, phi in zip(thetas, phis)]
        z_lst = [self.pivot_z - (self.r * np.cos(theta)) for theta in thetas]
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.scatter(x_lst, y_lst, z_lst)
        plt.show()

        # keys = points.keys()

        # find point closest to x = 0.5
        if len(x_lst) == 0:
            print("not enough points")
            return
        min_dist = (x_lst[0], 0)
        for i in range(len(x_lst)):
            point = x_lst[i]
            # print(point, type(point))
            if abs(point - 0.5) < abs(min_dist[0] - 0.5):
                min_dist = (point, i)

        index = min_dist[1]

        # to_go = PointStamped()
        # to_go.point.x, to_go.point.y, to_go.point.z = x_lst[index], y_lst[index], z_lst[index]

        goal = PoseStamped()
        goal.header.frame_id = "base"

        # goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = [0.551, -0.211, 0.177]

        goal.pose.position.x = x_lst[index] # 0.427
        goal.pose.position.y = y_lst[index] # -0.445
        goal.pose.position.z = z_lst[index] + 0.04 # 0.270

        self.angle = phis[-1]

        ### Need to find someway to get from angle -> euler angles -> quaternion
        # takes roll, pitch, yaw
        # q = tf.transformations.quaternion_from_euler(0, 0, self.angle)
        print("Angle: ", self.angle * 180 / np.pi)

        RY = np.array(tf.transformations.euler_matrix(0, np.pi, 0))
        RZ = np.array(tf.transformations.euler_matrix(0, 0, self.angle))
        q_matrix = RZ.dot(RY)
        # RY = np.array(tf.transformations.euler_matrix(0, -np.pi/2, 0))
        # RX = np.array(tf.transformations.euler_matrix(np.pi + self.angle, 0, 0))
        # q_matrix = tf.transformations.inverse_matrix(RX.dot(RY))
        q = tf.transformations.quaternion_from_matrix(q_matrix)

        print(q)
        q = normalize(q)
        goal.pose.orientation.x = q[0] # -0.006
        goal.pose.orientation.y = q[1] # -0.042
        goal.pose.orientation.z = q[2] # -0.696
        goal.pose.orientation.w = q[3] #0.717
        
        # if self.count == 500:
        #     self.pub.publish(to_go)
        # self.count += 1

        self.goal = goal
        self.subscriber.unregister()

        print("Done computing trajectory!")

def normalize(v):
    norm = np.linalg.norm(v)
    if norm == 0:
        return v
    return v / norm

# BASE_LOCATION_PARAM = "~location_topic/base"
# BASE_GOAL_PARAM = "~ball_goal"

def main():
    rospy.init_node("model")
    # if len(sys.argv) < 2:
    #     print("BAD BAD")
    
    # model = Model(sys.argv[0], sys.argv[1])
    BALL_LOC_TOPIC = "/base/ball/location"
    GOAL_TOPIC = "/ball/goal"
    model = Model(BALL_LOC_TOPIC, GOAL_TOPIC)
    while not rospy.is_shutdown():
        if model.goal is not None:
            model.pub.publish(model.goal)
            model.rate.sleep()

if __name__=='__main__':
    main()
