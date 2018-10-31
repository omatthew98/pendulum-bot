#!/usr/bin/python
from baxter_interface import CameraController
import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class BaxterImage:

	RIGHT_CAMERA_NAME = "right_hand_camera"
	LEFT_CAMERA_NAME = "left_hand_camera"
	HEAD_CAMERA_NAME = "head_camera"

	DEFAULT_CAMERA_RESOLUTION = (1280,800)

	lower_yellow = np.array([0, 125, 150])
	upper_yellow = np.array([255, 255, 255])

	def __init__(self, name):
		try:
			CameraController(BaxterImage.RIGHT_CAMERA_NAME).close()
		except:
			pass
		try:
			self.camera = CameraController(name)
			self.camera.open()
			self.camera.resolution = BaxterImage.DEFAULT_CAMERA_RESOLUTION
			self.bridge = CvBridge()
		except Exception as e:
			print(e)
			exit(1)

	def close(self):
		self.camera.close()

	def imgCallback(self, data):
		cvImage = self.bridge.imgmsg_to_cv2(data, "bgr8")
		# Thresholds for extracting the tennis ball
		# lower = [int(x) for x in input("Enter three lower numbers:").split(" ")]
		# upper = [int(x) for x in input("Enter three upper numbers:").split(" ")]
		mask = cv2.inRange(hsv, self.lower_yellow, self.upper_yellow)

		# Bitwis-AND mask and original image
		res = cv2.bitwise_and(cvImage, cvImage, mask=mask)
		blurred = cv2.medianBlur(res, 5)
		circles = (cv2.HoughCircles(blurred[:,:,2], cv2.HOUGH_GRADIENT, 1, 50,
           param1=50, param2=50, minRadius=0, maxRadius=0))

		circles = np.uint16(np.around(circles))
		for i in circles[0,:]:
			# draw the outer circle
			cv2.circle(img,(i[0],i[1]),i[2],(0,255,0),2)
			# draw the center of the circle
			cv2.circle(img,(i[0],i[1]),2,(0,0,255),3)
		cv2.imshow('frame', img)
		cv2.imshow('mask', mask)
		cv2.imshow('res', res)

if __name__ == "__main__":
	headCamera = BaxterImage(BaxterImage.HEAD_CAMERA_NAME)

	rospy.init_node('baxter_image')
	rospy.Subscriber("cameras/head_camera/image", Image, headCamera.imgCallback)

	rospy.spin()

