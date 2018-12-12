#!/usr/bin/python
import rospy
import numpy as np
import cv_bridge
import cv2 as cv
import imutils
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs.point_cloud2 import *
from sensor_msgs.point_cloud2 import _get_struct_fmt
from geometry_msgs.msg import PointStamped

# from baxter_imaging.msg import BallLoc

class KinectImage:
    def __init__(self, img_topic, depth_topic, color = "bgr8", CALIBRATE_COUNT=150.0):
        self.img_topic = img_topic
        self.depth_topic = depth_topic
        self.subscriber1 = rospy.Subscriber(img_topic, Image, self.imgCallback)
        self.subscriber2 = rospy.Subscriber(depth_topic, PointCloud2, self.depthCallback)
        self.bridge = cv_bridge.CvBridge()
        self.calibrate = 0
        self.CALIBRATE_COUNT = CALIBRATE_COUNT
        self.color = color
        self.background = cv.createBackgroundSubtractorMOG2()
        self.pub = rospy.Publisher("kinect/ball/location", PointStamped)
        self.rate = rospy.Rate(10)

    def imgCallback(self, data):
        if self.lastPC is None:
            return
        cvImage = self.bridge.imgmsg_to_cv2(data, "bgr8")
        # cv.ocl.setUseOpenCL(False)
        # # Calibrates the background for background subtraction
        # if self.calibrate < self.CALIBRATE_COUNT:
        #     if self.calibrate % 10 == 0:
        #         print("{}% Complete".format(self.calibrate * 100.0 / float(self.CALIBRATE_COUNT)))
        #     self.background.apply(cvImage)
        #     self.calibrate += 1
        #     return
        # # Calculate circles found
        # fgmask = self.background.apply(cvImage, learningRate = 0)
        # blurred = cv.medianBlur(fgmask, 5)
        # circles = cv.HoughCircles(blurred, cv.HOUGH_GRADIENT, 1, 200, param1 = 50, param2 = 25, minRadius = 0, maxRadius = 0)
        # if circles is not None:
        #     for i, circle in enumerate(circles[0:,]):
        #         if i > 4:
        #             break
        #         center = (circle[0][0], circle[0][1])
        #         radius = circle[0][2]
        #         depth = getDepth(center[0], center[1], self.lastPC)
        #         cv.circle(blurred, center, radius, (0, 255, 0), 2)
        #         cv.circle(blurred, center, 2, (0, 0, 255), 3)
        #         bl = BallLoc(center[0], center[1], depth, rospy.Time.now())
        #         self.pub.publish(bl)
        #         print(bl)
        # cv.imshow('circles', blurred)
        # cv.waitKey(1)

        # self.rate.sleep()
        greenLower = (29, 86, 6)
        greenUpper = (64, 255, 255)

        frame = cvImage
        blurred = cv.GaussianBlur(frame, (11, 11), 0)
        hsv = cv.cvtColor(blurred, cv.COLOR_BGR2HSV)

        mask = cv.inRange(hsv, greenLower, greenUpper)
        mask = cv.erode(mask, None, iterations=2)
        mask = cv.dilate(mask, None, iterations=2)

        cnts = cv.findContours(mask.copy(), cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        # print("Center:",cnts)
        cnts = grab_contours(cnts)
        center = None

        # only proceed if at least one contour was found
        if len(cnts) > 0:
		    # find the largest contour in the mask, then use
		    # it to compute the minimum enclosing circle and
		    # centroid
            c = max(cnts, key=cv.contourArea)
            ((x, y), radius) = cv.minEnclosingCircle(c)
            M = cv.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
 
            # only proceed if the radius meets a minimum size
            if radius > 10:
                # draw the circle and centroid on the frame,
                # then update the list of tracked points
                cv.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                cv.circle(frame, center, 5, (0, 0, 255), -1)
                p = PointStamped()
                p.point.x, p.point.y, p.point.z = getDepth(x, y, self.lastPC)
                p.header.frame_id = "kinect_frame"
                
                self.pub.publish(p)
                print(p)

        cv.imshow("Frame", frame)
        key = cv.waitKey(1) & 0xFF

    def depthCallback(self, pc):
        self.lastPC = pc

# def getDepth(x, y, lPC, numSamples=100, mu=0, sigma=7):
#     pts = np.array([])
#     for _ in range(numSamples):
#         try:
#             dx, dy = np.random.normal(mu, sigma, 2)
#             pt = read_point(int(np.round(x + dx)), int(np.round(y + dy)), lPC)[2]
#             if not np.isnan(pt):
#                 pts = np.append(pts, pt)      
#         except Exception as e:
#             print(_, "failed:", e)
#             pass
#     return np.median(pts)


def getDepth(x, y, lPC, numSamples=100, mu=0, sigma=7):
    ptsX, ptsY, ptsD = np.array([]), np.array([]), np.array([])
    for _ in range(numSamples):
        try:
            dx, dy = np.random.normal(mu, sigma, 2)
            pt = read_point(int(np.round(x + dx)), int(np.round(y + dy)), lPC)

            if not np.isnan(pt[0]):
                ptsX = np.append(ptsX, pt[0])
            if not np.isnan(pt[1]):
                ptsY = np.append(ptsY, pt[1])
            if not np.isnan(pt[2]):
                ptsD = np.append(ptsD, pt[2])      
        except Exception as e:
            print(_, "failed:", e)
            pass
    return np.mean(ptsX), np.mean(ptsY), np.median(ptsD)

def read_point(x, y, cloud, field_names=None, skip_nans=False, uvs=[]):
    assert isinstance(cloud, roslib.message.Message) and cloud._type == 'sensor_msgs/PointCloud2', 'cloud is not a sensor_msgs.msg.PointCloud2'
    fmt = _get_struct_fmt(cloud.is_bigendian, cloud.fields, field_names)
    width, height, point_step, row_step, data, isnan = cloud.width, cloud.height, cloud.point_step, cloud.row_step, cloud.data, math.isnan
    unpack_from = struct.Struct(fmt).unpack_from
    offset = row_step * y + point_step * x
    return unpack_from(data, offset)

# Code from: https://github.com/jrosebr1/imutils/blob/5aae9887df3dcada5f8d8fa6af0df2122ad7aaca/imutils/convenience.py
# This is code from the imutils package that for whatever reason does not appear to 
# be in the version on the lab computers. 
def grab_contours(cnts):
    # if the length the contours tuple returned by cv2.findContours
    # is '2' then we are using either OpenCV v2.4, v4-beta, or
    # v4-official
    if len(cnts) == 2:
        cnts = cnts[0]
    # if the length of the contours tuple is '3' then we are using
    # either OpenCV v3, v4-pre, or v4-alpha
    elif len(cnts) == 3:
        cnts = cnts[1]
    # otherwise OpenCV has changed their cv2.findContours return
    # signature yet again and I have no idea WTH is going on
    else:
        raise Exception(("Contours tuple must have length 2 or 3, "
            "otherwise OpenCV changed their cv2.findContours return "
            "signature yet again. Refer to OpenCV's documentation "
            "in that case"))
    # return the actual contours array
    return cnts

def main():
    rospy.init_node('kinect_image')
    rgb = KinectImage('/camera/rgb/image_raw', '/camera/depth/points')
    rospy.spin()

if __name__=='__main__':
    main()