#!/usr/bin/python
import os
import sys

import rospy

import cv2
import cv_bridge

from sensor_msgs.msg import Image

def send_image(path):
    img = cv2.imread(path)
    msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8")
    pub = rospy.Publisher('/robot/xdisplay', Image, latch=True)
    pub.publish(msg)
    rospy.sleep(1)

def main():
    rospy.init_node('rsdk_xdisplay_image')
    img_path = rospy.get_param("image_path")
    if not img_path:
        rospy.logerr("No image_path given.")
        return 1
    elif not os.access(img_path, os.R_OK):
        rospy.logerr("Cannot read file at '%s'" % (img_path,))
        return 1
    send_image(img_path)
    return 0

if __name__ == '__main__':
    sys.exit(main())
