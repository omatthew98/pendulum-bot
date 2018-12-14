#!/usr/bin/python
import rospy
import numpy as np
import cv_bridge
import cv2 as cv
import imutils
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from sensor_msgs.point_cloud2 import *
from sensor_msgs.point_cloud2 import _get_struct_fmt
from geometry_msgs.msg import PointStamped

RGB_TOPIC_PARAM="~camera_topic/rgb"
DEPTH_TOPIC_PARAM="~camera_topic/depth"
LOCATION_TOPIC_PARAM="~location_topic/kinect"
CAMERA_INFO_TOPIC_PARAM="~camera_topic/info"
BALL_FRAME_PARAM="~frame/ball"


class KinectImage:
    def __init__(self, img_topic, depth_topic, pub_topic, camera_info_topic, ball_frame, CALIBRATE_COUNT=150.0):
        self.lastPC = None

        self.bridge = cv_bridge.CvBridge()
        self.background = cv.createBackgroundSubtractorMOG2()

        self.calibrate = 0
        self.CALIBRATE_COUNT = CALIBRATE_COUNT
        self.calibrated = False
        self.clicks = []

        self.rate = rospy.Rate(10)

        self.ball_frame = ball_frame
        try:
            cam_info = rospy.wait_for_message(camera_info_topic, CameraInfo, timeout=10)
        except:
            rospy.signal_shutdown("Error trying to get camera information")
        self.maxX, self.maxY = cam_info.width, cam_info.height
        self.minX, self.minY = 0, 0
        self.subscriber1 = rospy.Subscriber(img_topic, Image, self.imgCallback)
        self.subscriber2 = rospy.Subscriber(depth_topic, PointCloud2, self.depthCallback)
        self.pub = rospy.Publisher(pub_topic, PointStamped)

    def clickCallback(self, event, x, y, flag, param):
        if (event == cv.EVENT_LBUTTONUP):
            point = (x, y)
            self.clicks.append(point)

    def imgCallback(self, data):
        while not self.calibrated:
            cvImage = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.clicks = []
            cv.imshow("Choose Area of Interest", cvImage)
            while len(self.clicks) < 4:
                cv.setMouseCallback("Choose Area of Interest", self.clickCallback)
                key = cv.waitKey(10)
                if key == ord(' '):
                    rospy.loginfo("Using default")
                    self.clicks = []
                    self.calibrated = True
                    cv.destroyAllWindows()
                    return
            cv.destroyAllWindows()
            for point in self.clicks:
                cv.circle(cvImage, point, 3, (0, 255, 0), -1)
            cv.imshow("Confirm selection", cvImage)
            key = cv.waitKey(0)
            if key == ord(' '):
                self.minX = min([xy[0] for xy in self.clicks])
                self.minY = min([xy[1] for xy in self.clicks])
                self.maxX = max([xy[0] for xy in self.clicks])
                self.maxY = max([xy[1] for xy in self.clicks])
                rospy.loginfo("Calibrated X: %d - %d Y: %d - %d", self.minX, self.maxX, self.minY, self.maxY)
                self.calibrated = True
                cv.destroyAllWindows()
                return
            else:
                cv.destroyAllWindows()
        if self.lastPC is None:
            return
        cvImage = self.bridge.imgmsg_to_cv2(data, "bgr8")
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
        valid_cnts = []
        for cnt in grab_contours(cnts):
            ((x, y), radius) = cv.minEnclosingCircle(cnt)
            if x >= self.minX and x <= self.maxX and y >= self.minY and y <= self.maxY:
                valid_cnts.append(cnt)
        center = None

        # only proceed if at least one contour was found
        if len(valid_cnts) > 0:
		    # find the largest contour in the mask, then use
		    # it to compute the minimum enclosing circle and
		    # centroid
            c = max(valid_cnts, key=cv.contourArea)
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
                p.point.z, p.point.y, p.point.x = getDepth(x, y, self.lastPC)
                # p.point.x, p.point.y, p.point.z = x * 0.001, y * 0.001, getDepth(x, y, self.lastPC)[2]
                p.header.frame_id = self.ball_frame
                
                self.pub.publish(p)

        cv.imshow("Frame", frame)
        key = cv.waitKey(1) & 0xFF

    def depthCallback(self, pc):
        self.lastPC = pc
        
def getDepth(x, y, lPC, numSamples=100, mu=0, sigma=7):
    ptsX, ptsY, ptsD = np.array([]), np.array([]), np.array([])
    for _ in range(numSamples):
        try:
            dx, dy = np.random.normal(mu, sigma, 2)
            # pts_gen = read_points(lPC, uvs=[(int(np.round(x + dx)), int(np.round(y + dy)))])
            # pt = next(pts_gen)
            pt = read_point(int(np.round(x + dx)), int(np.round(y + dy)), lPC)
            if not np.isnan(pt[0]):
                ptsX = np.append(ptsX, pt[0])
            if not np.isnan(pt[1]):
                ptsY = np.append(ptsY, pt[1])
            if not np.isnan(pt[2]):
                ptsD = np.append(ptsD, pt[2])      
        except Exception as e:
            rospy.logerr("failed: %s", e)
            pass
    return np.median(ptsX), np.median(ptsY), np.median(ptsD)

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
    rospy.init_node('kinect_image', log_level=rospy.DEBUG)
    if not rospy.has_param(RGB_TOPIC_PARAM):
        rospy.logerr("Couldn't find parameter %s", RGB_TOPIC_PARAM)
        exit(1)
    if not rospy.has_param(DEPTH_TOPIC_PARAM):
        rospy.logerr("Couldn't find parameter %s", DEPTH_TOPIC_PARAM)
        exit(1)
    if not rospy.has_param(LOCATION_TOPIC_PARAM):
        rospy.logerr("Couldn't find parameter %s", LOCATION_TOPIC_PARAM)
        exit(1)
    if not rospy.has_param(BALL_FRAME_PARAM):
        rospy.logerr("Couldn't find parameter %s", BALL_FRAME_PARAM)
        exit(1)
    if not rospy.has_param(CAMERA_INFO_TOPIC_PARAM):
        rospy.logerr("Couldn't find parameter %s", CAMERA_INFO_TOPIC_PARAM)
        exit(1)
    rgb = KinectImage( 
        rospy.get_param(RGB_TOPIC_PARAM), 
        rospy.get_param(DEPTH_TOPIC_PARAM), 
        rospy.get_param(LOCATION_TOPIC_PARAM), 
        rospy.get_param(CAMERA_INFO_TOPIC_PARAM),
        rospy.get_param(BALL_FRAME_PARAM))
    rospy.spin()
    rgb.subscriber1.unregister()
    rgb.subscriber2.unregister()

if __name__=='__main__':
    main()