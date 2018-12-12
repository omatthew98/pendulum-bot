#!/usr/bin/env python

import rospy
import numpy as np
import cv_bridge
import cv2 as cv
import imutils
from image_geometry import PinholeCameraModel
import tf

from baxter_imaging.msg import BallLoc
from sensor_msgs.msg import CameraInfo

class Transformer():

    def __init__(self, sub_topic, sub_topic2, cam1_frame_id, cam2_frame_id):
        self.listener = tf.TransformListener()
        self.cam1_frame_id, self.cam2_frame_id = cam1_frame_id, cam2_frame_id
        self.poses = {}
        self.transformation = None
        self.sub = rospy.Subscriber(sub_topic, BallLoc, self.callback)
        # self.pub = rospy.Publisher(pub_topic, BallLoc)
        self.cam = PinholeCameraModel()
        self.sub2 = rospy.Subscriber(sub_topic2, CameraInfo, self.callback2)
        


    def callback(self, loc):
        x, y, z = loc.x, loc.y, loc.d

        print(self.cam.projectPixelTo3dRay((350, 415)))

    def callback2(self, info):
        self.cam.fromCameraInfo(info)
        # print(self.cam.cx())


def main():
    rospy.init_node('coord_transform')
    rgb = Transformer('/kinect/ball/location', '/camera/rgb/camera_info', None, None)
    rospy.spin()



if __name__ == "__main__":
    main()