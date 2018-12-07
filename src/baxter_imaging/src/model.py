#!/usr/bin/python
import numpy as np
import sys
import rospy
import tf
import collections

from baxter_imaging.msg import BallLoc
from scipy.integrate import solve_ivp
from geometry_msgs.msg import PointStamped

from baxter_interface import Limb
import moveit_commander
from baxter_interface import gripper as robot_gripper

class Model:

    def __init__(self, topic):
        self.topic, self.length = topic, 20
        self.xdata, self.ydata, self.zdata = [], [], []
        self.xc, self.yc, self.zc = 0, 0, 0
        self.calibrated = False
        self.count = 0
        self.calibration = rospy.Subscriber("/world/ball/calibration", PointStamped, self.initiate)
        self.subscriber = rospy.Subscriber(topic, PointStamped, self.callback)
        self.pub = rospy.Publisher("kinect/ball/goal", PointStamped)
        self.rate = rospy.Rate(10)       
        self.queue = collections.deque(maxlen = 10)
        self.r = 1.2
        self.group = moveit_commander.MoveGroupCommander("right_arm")
        self.pose = self.group.get_current_pose()
        self.gripper = robot_gripper.Gripper('right')
        self.gripper.set_velocity(100)
        self.gripper.calibrate()
        rospy.sleep(2)
        self.gripper.open()

    def initiate(self, data):
        if not self.calibrated:
            self.xc, self.yc, self.zc = data.point.x, data.point.y, data.point.z
            self.calibrated = True

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

    def gripper_handler(self, x, y, z):

        x_end, y_end, z_end = self.pose.pose.position.x, self.pose.pose.position.y, self.pose.pose.position.z

        threshold = np.sqrt((x - x_end)**2 + (y - y_end)**2 + (z - z_end)**2)

        print(threshold)

        self.queue.append(threshold)

        for t in self.queue:
            if t > 1.3:
                return
            
        print("Threshold reached")
        self.gripper.close()

        rospy.sleep(2)

        self.gripper.open()

        self.queue.clear()

        # rospy.sleep(2)

    def callback(self, data):

        # print("Model got location: ", data)
        x, y, z = data.point.x, data.point.y, data.point.z
        # self.gripper_handler(x, y, z)

        theta = np.arccos(float(self.zc - z) / self.r)

        if (x == 0 and y == 0) or (x == self.xc) or (y == self.yc):
            phi = 0
        elif x == 0:
            if y > 0:
                phi = np.pi / 2
            else:
                phi = -1 * np.pi / 2
        elif y == 0:
            phi = 0
        else:
            phi = np.arctan(float(y - self.yc) / (x - self.xc))

        sol = solve_ivp(self.system, [0, 10], [theta, 0, phi, 0], max_step = 0.1)

        thetas = sol.y[0]
        phis = sol.y[2]
        angles = zip(thetas, phis)

        # points = {-self.r * np.cos(theta) : (self.r * np.sin(theta) * np.cos(phi), self.r * np.sin(theta) * np.sin(phi)) for theta, phi in zip(thetas, phis)}
        # print(points)

        x_lst = [(self.xc + (self.r * np.sin(theta) * np.cos(phi))) for theta, phi in zip(thetas, phis)]
        y_lst = [(self.yc + (self.r * np.sin(theta) * np.sin(phi))) for theta, phi in zip(thetas, phis)]
        z_lst = [(self.zc - (self.r * np.cos(theta))) for theta in thetas]

        # keys = points.keys()

        index = int(len(x_lst) * 3/4)

        to_go = PointStamped()
        to_go.point.x, to_go.point.y, to_go.point.z = x_lst[index], y_lst[index], z_lst[index]
        
        if self.count == 500:
            self.pub.publish(to_go)
        self.count += 1

        print(self.count)

def main():
    rospy.init_node('model')
    model = Model("world/ball/location")
    rospy.spin()

if __name__=='__main__':
    main()