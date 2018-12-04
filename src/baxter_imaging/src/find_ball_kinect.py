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
greenLower = (21, 69, 107)
greenUpper = (255, 255, 255)

class KinectImage:
    def __init__(self, img_topic, depth_topic, color = "bgr8"):
        self.img_topic = img_topic
        self.depth_topic = depth_topic
        self.subscriber1 = rospy.Subscriber(img_topic, Image, self.imgCallback)
        self.subscriber2 = rospy.Subscriber(depth_topic, Image, self.depthCallback)
        self.bridge = cv_bridge.CvBridge()
        self.calibrate = 0
        self.color = color
        self.background = cv.createBackgroundSubtractorMOG2()
        self.pub1 = rospy.Publisher("circ_pos", Circ)
        self.pub2 = rospy.Publisher("circ_depth", Float32)
        self.rate = rospy.Rate(10)
        self.path = []
        self.calibration_center = []
        self.calibrated = False
        self.tracking = True
        self.thetas = []
        self.l = 0



    def imgCallback(self, data):
        cvImage = None
        if self.color:
            cvImage = self.bridge.imgmsg_to_cv2(data, "bgr8")
        else:
            cvImage = self.bridge.imgmsg_to_cv2(data)
        cv.ocl.setUseOpenCL(False)
        if self.calibrate < CALIBRATE_COUNT:
            if self.calibrate % 10 == 0:
                print("{}% Complete".format(self.calibrate * 100.0 / float(CALIBRATE_COUNT)))
            self.background.apply(cvImage)
            self.calibrate += 1
            return

        cv.imshow(self.img_topic, cvImage)

        frame = cvImage

        # frame = imutils.resize(cvImage, width=600)
        blurred = cv.GaussianBlur(cvImage, (11, 11), 0)
        hsv = cv.cvtColor(blurred, cv.COLOR_BGR2HSV)

        mask = cv.inRange(hsv, greenLower, greenUpper)
        mask = cv.erode(mask, None, iterations = 2)
        mask = cv.dilate(mask, None, iterations = 2)

        cnts = cv.findContours(mask.copy(), cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        cnts = cnts[0] if imutils.is_cv2() else cnts[1]
        center = None

        if len(cnts) > 0:
            c = max(cnts, key = cv.contourArea)
            ((x, y), radius) = cv.minEnclosingCircle(c)
            M = cv.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            if radius > 10:
                cv.circle(cvImage, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                # cv.circle(frame, center, 5, (0, 0, 255), -1)

            c = Circ(x, y, radius)
            self.pub.publish(c)

        if self.calibrated and center:
            calibrated_point = (center[0] - self.calibration_center[0], center[1] - self.calibration_center[1])
            # c = Circ(calibrated_point[0], calibrated_point[1], radius)
            # self.pub.publish(c)
            if self.tracking:
                self.path.append(calibrated_point)
            x = self.l - abs(calibrated_point[1])
            r = self.l
            print("Point: ", calibrated_point)
            z = float(x) / float(r)
            theta = (np.arccos(z)) * 180 / np.pi
            print("Theta: ", x, r, theta)
            self.thetas.append(theta)

        key = cv.waitKey(1) & 0xFF

        if key == ord("q"):
            pass

        elif key == ord("c"):
            self.calibration_center = [center[0], center[1]]
            self.l = abs(self.calibration_center[1])
            self.calibrated = True

        elif key == ord("t"):
            if not calibrated:
                pass
            else:
                self.tracking = True


        # fgmask = self.background.apply(cvImage, learningRate = 0)
        # blurred = cv.medianBlur(fgmask, 5)
        # circles = cv.HoughCircles(blurred, cv.HOUGH_GRADIENT, 1, 200, param1 = 50, param2 = 25, minRadius = 0, maxRadius = 0)
        # if circles is not None:
        #     for i, circle in enumerate(circles[0:,]):
        #         if i > 4:
        #             break
        #         center = (circle[0][0], circle[0][1])
        #         radius = circle[0][2]
        #         # cv.circle(blurred, center, radius, (0, 255, 0), 2)
        #         # cv.circle(blurred, center, 2, (0, 0, 255), 3)
        #         c = Circ(center[0], center[1], radius)
        #         self.pub.publish(c)
        #         print(c)
        #     # cv.imshow('circles', blurred)
        #     # cv.waitKey(0)
        cv.imshow(self.img_topic, cvImage)
        cv.waitKey(1)
        self.rate.sleep()

    def depthCallback(self, data):

        cvImage = self.bridge.imgmsg_to_cv2(data)

        x = int(self.circ.x)
        y = int(self.circ.y)
        self.pub2.publish(cvImage[x][y])
        cv.circle(cvImage, (x, y), 5, (255, 0, 0), -1)
        cv.imshow(self.topic, cvImage)
        cv.waitKey(1)
        self.rate.sleep()




def main():
    rospy.init_node('kinect_image')
    rgb = KinectImage('/camera/rgb/image_raw', '/camera/depth/image_raw')
    rospy.spin()

if __name__=='__main__':
    main()