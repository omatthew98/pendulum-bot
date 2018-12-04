#!/usr/bin/python
import rospy
import numpy as np
import cv_bridge
import cv2 as cv
import imutils
from sensor_msgs.msg import Image
from std_msgs.msg import Float32

from baxter_imaging.msg import Circ

CALIBRATE_COUNT = 150.0

class DepthImage():
    def __init__(self, topic):
        self.topic = topic
        rospy.Subscriber("circ_pos", Circ, self.callback1)
        rospy.Subscriber(topic, Image, self.callback2)
        self.bridge = cv_bridge.CvBridge()
        self.calibrate = 0
        self.background = cv.createBackgroundSubtractorMOG2()
        self.pub = rospy.Publisher("circ_depth", Float32)
        self.rate = rospy.Rate(10)
        self.circ = None

    def callback1(self, data):
        self.circ = data
        # print("Want depth at point: ", self.circ.x, self.circ.y)

    def callback2(self, data):
        cvImage = self.bridge.imgmsg_to_cv2(data)
        # cv.ocl.setUseOpenCL(False)c
        # if self.calibrate < CALIBRATE_COUNT:
        #     if self.calibrate % 10 == 0:
        #         print("{}% Complete".format(self.calibrate * 100.0 / float(CALIBRATE_COUNT)))
        #     self.background.apply(cvImage)
        #     self.calibrate += 1
        #     return

        x = int(self.circ.x)
        y = int(self.circ.y)
        self.pub.publish(cvImage[x][y])
        # cv.circle(cvImage, (x, y), 5, (255, 0, 0), -1)
        cv.imshow(self.topic, cvImage)
        cv.waitKey(1)
        self.rate.sleep()


def main():
    rospy.init_node('ball_depth')
    depth = DepthImage('/camera/depth/image_raw')
    rospy.spin()

if __name__=='__main__':
    main()