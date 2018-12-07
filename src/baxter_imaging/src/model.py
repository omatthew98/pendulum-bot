#!/usr/bin/python
import numpy as np
import sys
import rospy

from baxter_imaging.msg import BallLoc
from scipy.integrate import solve_ivp
from geometry_msgs.msg import PointStamped

class Model:

    def __init__(self, topic):
        self.topic = topic
        self.subscriber = rospy.Subscriber(topic, PointStamped, self.callback)
        self.pub = rospy.Publisher("kinect/ball/goal", BallLoc)
        self.rate = rospy.Rate(10)
        self.r = 5

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
        print("Model got location: ", data)
        x, y, z = data.x, data.y, data.d
        theta = np.arccos(z / self.r)

        if x == 0 and y == 0:
            phi = 0
        elif x == 0:
            if y > 0:
                phi = np.pi / 2
            else:
                phi = -1 * np.pi / 2
        elif y == 0:
            phi = 0
        else:
            phi = np.arctan(y / x)

        sol = solve_ivp(self.system, [0, 10], [theta, 0, phi, 0], max_step = 0.1)

        thetas = sol.y[0]
        phis = sol.y[2]
        angles = zip(thetas, phis)

        points = {-self.r * np.cos(theta) : (self.r * np.sin(theta) * np.cos(phi), self.r * np.sin(theta) * np.sin(phi)) for theta, phi in zip(thetas, phis)}
        # print(points)
        # x = [r * np.sin(theta) * np.cos(phi) for theta, phi in zip(thetas, phis)]
        # y = [r * np.sin(theta) * np.sin(phi) for theta, phi in zip(thetas, phis)]
        # z = [-r * np.cos(theta) for theta in thetas]

        keys = points.keys()
        
        goal_point = points[keys[0]]
        goal_loc = BallLoc(goal_point[0], goal_point[1], 1, rospy.Time())
        # print(goal_loc)
        self.pub.publish(goal_loc)

def main():
    rospy.init_node('model')
    model = Model("world/ball/location")
    rospy.spin()

if __name__=='__main__':
    main()