#!/usr/bin/python
from baxter_interface import CameraController
import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


RIGHT_CAMERA_NAME = "right_hand_camera"
LEFT_CAMERA_NAME = "left_hand_camera"
HEAD_CAMERA_NAME = "head_camera"

BRIDGE = CvBridge()

DEFAULT_CAMERA_RESOLUTION = (1280,800)

def imageCallback(msg):
	cv_image = BRIDGE.imgmsg_to_cv2(msg, "bgr8")
	cv2.imshow('test image', cv_image)
	cv2.waitKey(0)
	cv2.destroyAllWindows()
	rospy.sleep(1.)

if __name__ == "__main__":
	try:
		c = CameraController(RIGHT_CAMERA_NAME)
		print("Closing {}.".format(RIGHT_CAMERA_NAME))
		c.close()
		print("Closed.")
	except Exception as e:
		pass
	try:
		left_camera = CameraController(LEFT_CAMERA_NAME)
		head_camera = CameraController(HEAD_CAMERA_NAME)
		print("Got cameras.")
		print("Opening {}.".format(LEFT_CAMERA_NAME))
		left_camera.open()
		print("Opened.")
		print("Opening {}.".format(HEAD_CAMERA_NAME))
		head_camera.open()
		print("Opened.")
		print("Setting resolutions to {}".format(DEFAULT_CAMERA_RESOLUTION))
		left_camera.resolution = DEFAULT_CAMERA_RESOLUTION
		head_camera.resolution = DEFAULT_CAMERA_RESOLUTION
	except Exception as e:
		print(e)
		exit(1)

	rospy.init_node('baxter_image')
	rospy.Subscriber("cameras/head_camera/image", Image, imageCallback)

	

	rospy.spin()

