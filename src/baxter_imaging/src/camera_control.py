#!/usr/bin/python
from baxter_interface import CameraController
import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

MARGIN_FOR_COLOR_BLUE = 20
MARGIN_FOR_COLOR_GREEN = 50
MARGIN_FOR_COLOR_RED = 10


class BaxterImage:

	RIGHT_CAMERA_NAME = "right_hand_camera"
	LEFT_CAMERA_NAME = "left_hand_camera"
	HEAD_CAMERA_NAME = "head_camera"

	MIN_CALIBRATION = 10

	DEFAULT_CAMERA_RESOLUTION = (1280,800)

	lower_yellow = np.array([0, 125, 150])
	upper_yellow = np.array([255, 255, 255])

	def __init__(self, name):
		try:
			print("Closing {}".format(BaxterImage.HEAD_CAMERA_NAME))
			CameraController(BaxterImage.HEAD_CAMERA_NAME).close()
		except:
			pass
		try:
			print("Initializing BaxterImage...")
			self.name = name
			self.camera = CameraController(name)
			self.camera.open()
			self.camera.resolution = BaxterImage.DEFAULT_CAMERA_RESOLUTION
			self.bridge = CvBridge()
			self.calibrationCount = 0
			self.calibrations = []
			print("BaxterImage initialized.")
		except Exception as e:
			print(e)
			exit(1)

	def close(self):
		self.camera.close()

	def imgCallback(self, data):
		cvImage = self.bridge.imgmsg_to_cv2(data, "bgr8")

		if self.calibrationCount < self.MIN_CALIBRATION:
			cv2.namedWindow('calibration')
			cv2.setMouseCallback('calibration', self.calibrateCB)

			while self.calibrationCount < self.MIN_CALIBRATION:
				cv2.imshow('calibration', cvImage)
				key = cv2.waitKey(1) & 0xFF

				if key == ord('\n'):
					break
			cv2.destroyAllWindows()
			self.calibrate(cvImage)

		else:
			# Thresholds for extracting the tennis ball
			# lower = [int(x) for x in input("Enter three lower numbers:").split(" ")]
			# upper = [int(x) for x in input("Enter three upper numbers:").split(" ")]
			hsv = cv2.cvtColor(cvImage, cv2.COLOR_BGR2HSV)
			mask = cv2.inRange(hsv, self.lower_yellow, self.upper_yellow)

			# Bitwis-AND mask and original image
			res = cv2.bitwise_and(cvImage, cvImage, mask=mask)
			blurred = cv2.medianBlur(res, 5)
			circles = (cv2.HoughCircles(blurred[:,:,2], cv2.HOUGH_GRADIENT, 1, 50,
	           param1=50, param2=50, minRadius=0, maxRadius=0))
			print(circles)
			# circles = np.uint16(np.around(circles))
			# for i in circles[0,:]:
			# 	# draw the outer circle
			# 	cv2.circle(cvImage,(i[0],i[1]),i[2],(0,255,0),2)
			# 	# draw the center of the circle
			# 	cv2.circle(cvImage,(i[0],i[1]),2,(0,0,255),3)
			cv2.imshow('frame', cvImage)
			cv2.imshow('hsv', hsv)
			cv2.imshow('mask', mask)
			cv2.imshow('res', res)
			cv2.waitKey(0)
			cv2.destroyAllWindows()

	def calibrateCB(self, event, x, y, flags, param):
		if event == cv2.EVENT_LBUTTONDOWN:
			print("Click recorded for {}, {}".format(x, y))
			self.calibrations.append((x, y))
			self.calibrationCount += 1

	def calibrate(self, img):
		minB, maxB = 255, 0
		minG, maxG = 255, 0
		minR, maxR = 255, 0
		for (x, y) in self.calibrations:
			minB = min(minB, img[x,y][0])
			minG = min(minG, img[x,y][1])
			minR = min(minR, img[x,y][2])
			maxB = max(maxB, img[x,y][0])
			maxG = max(maxG, img[x,y][1])
			maxR = max(maxR, img[x,y][2])
		colorLower = [  max(0, minB - MARGIN_FOR_COLOR_BLUE), 
						max(0, minG - MARGIN_FOR_COLOR_GREEN), 
						max(0, minR - MARGIN_FOR_COLOR_RED)
					]
		colorUpper = [min(255, maxB + MARGIN_FOR_COLOR_BLUE), 
						min(255, maxG + MARGIN_FOR_COLOR_GREEN), 
						min(255, maxR + MARGIN_FOR_COLOR_RED)
					]
		hsvLower = cv2.cvtColor(np.uint8([[colorLower]]), cv2.COLOR_BGR2HSV)
		hsvUpper = cv2.cvtColor(np.uint8([[colorUpper]]), cv2.COLOR_BGR2HSV)
		self.lower_yellow = np.array(hsvLower[0][0])
		self.upper_yellow = np.array(hsvUpper[0][0])
		print(self.lower_yellow)
		print(self.upper_yellow)


if __name__ == "__main__":
	camera = BaxterImage(BaxterImage.RIGHT_CAMERA_NAME)

	rospy.init_node('baxter_image')
	rospy.Subscriber("cameras/{}/image".format(camera.name), Image, camera.imgCallback)

	rospy.spin()

