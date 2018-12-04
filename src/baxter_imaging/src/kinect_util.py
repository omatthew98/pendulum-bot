#!/usr/bin/python
import rospy
import numpy as np
import cv_bridge
import cv2 as cv
import imutils
from sensor_msgs.msg import Image, PointCloud2
import sensor_msgs.point_cloud2 as pc2

from baxter_imaging.msg import Circ



class KinectImage:
    def __init__(self, img_topic, depth_topic):
        self.img_topic = img_topic
        self.depth_topic = depth_topic
        self.subscriber1 = rospy.Subscriber(img_topic, Image, self.imgCallback)
        self.subscriber2 = rospy.Subscriber(depth_topic, PointCloud2, self.depthCallback)
        self.lastPC = None
        self.bridge = cv_bridge.CvBridge()
        self.pub1 = rospy.Publisher("circ_pos", Circ)
        # self.pub2 = rospy.Publisher("circ_depth", Float32)
        self.rate = rospy.Rate(10)

    def imgCallback(self, data):
        if self.lastPC is None:
            return
        cvImage = self.bridge.imgmsg_to_cv2(data, "bgr8")
        cv.ocl.setUseOpenCL(False)
        def clickCallback(event, x, y, flags, param):
            lPC = self.lastPC
            if event == cv.EVENT_LBUTTONDOWN:
                print(lPC)
                curPC = [x for x in pc2.read_points(lPC)]
                print(curPC[x + y * lPC.width])
        cv.namedWindow(self.img_topic)
        cv.setMouseCallback(self.img_topic, clickCallback)
        cv.imshow(self.img_topic, cvImage)
        cv.waitKey(1)
        self.rate.sleep()

    def depthCallback(self, pc):
        self.lastPC = pc

def main():
    rospy.init_node('kinect_image')
    rgb = KinectImage('/camera/rgb/image_raw', '/camera/depth/points')
    rospy.spin()

if __name__=='__main__':
    main()