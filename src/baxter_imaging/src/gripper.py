#!/usr/bin/python
import sys
import rospy
import numpy as np
import moveit_commander
import collections
import tf

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from baxter_interface import Limb
from baxter_interface import gripper as robot_gripper

from geometry_msgs.msg import PointStamped

THRESHOLD = 0.7

class Gripper:

	def __init__(self, sub_topic1, sub_topic2):

		self.ready = False

		self.rate = rospy.Rate(10)	   
		# self.queue = collections.deque(maxlen = 10)
		# Generalize this to both arms?
		self.group = moveit_commander.MoveGroupCommander("right_arm")
		self.listener = tf.TransformListener()
		self.gripper = robot_gripper.Gripper('right')
		self.gripper.set_velocity(100)
		self.gripper.calibrate()
		rospy.sleep(2)
		self.gripper.open()

		self.sub_ready = rospy.Subscriber(sub_topic1, Bool, self.check_ready)
		self.sub_ball_pos = rospy.Subscriber(sub_topic2, PointStamped, self.grip)

	def check_ready(self, ready):
		if ready.data:
			self.ready = True

	def grip(self, ball_pos):
		if not self.ready:
			return
		x, y, z = ball_pos.point.x, ball_pos.point.y, ball_pos.point.z
		trans, rot = self.listener.lookupTransform("/reference/base", "/reference/right_hand", rospy.Time())

		x_end, y_end, z_end = trans
		z_end -= 0.04
		threshold = np.sqrt((x - x_end)**2 + (y - y_end)**2 + (z - z_end)**2)
		print(threshold)
		# self.queue.append(threshold)
		# for t in self.queue:
		if threshold > THRESHOLD:
			return
		print("Threshold reached")
		self.gripper.close()
		rospy.sleep(1)
		self.gripper.open()
		# self.queue.clear()
		# rospy.sleep(2)

def main():
	rospy.init_node("gripper_node")
	BALL_GOAL = "ball/ready"
	BALL_LOCATION_PARAM = "/base/ball/location"
	act = Gripper(BALL_GOAL, BALL_LOCATION_PARAM)
	rospy.spin()
	
if __name__ == "__main__":
	main()
