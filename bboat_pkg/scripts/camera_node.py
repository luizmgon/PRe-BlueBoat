#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped, Point

from time import sleep

from mavros_msgs.msg import State, OverrideRCIn

from lib.bboat_lib import *

class CameraNode():
	'''
		Camera Node
		Drives the camera to the vSB heading angle

	'''
	def __init__(self):

		self.heading_rob = 0
		self.heading_vSB = 0


		self.rate = rospy.Rate(50)

		# --- Subs
		self.sub_pose_robot = rospy.Subscriber('/pose_robot_R0', PoseStamped, self.Pose_Robot_callback)
		# rospy.wait_for_message('/pose_robot_R0', PoseStamped, timeout=None)

		self.sub_vsb_pose = rospy.Subscriber('/vSBPosition', PoseStamped, self.Pose_vSB_callback)

		# --- Pubs
		self.pub_cam_cmd = rospy.Publisher('/cam_cmd', Float64, queue_size=10)

		# --- Init done
		rospy.loginfo('[CAMERA] Camera Node Start')

	def loop(self): 
		# rospy.loginfo('[CAMERA] Camera node loop')

		while not rospy.is_shutdown():

			theta_cam = sawtooth(self.heading_rob - self.heading_vSB)
			
			if abs(theta_cam) > pi/2: 
				theta_cam = sign(theta_cam)*pi/2

			self.pub_cam_cmd.publish(theta_cam)

			self.rate.sleep()



	def Pose_Robot_callback(self, msg):
		'''
			Parse robot pose message - x, y, psi in local lambert frame R0
		'''
		self.heading_rob = msg.pose.position.z # psi

	def Pose_vSB_callback(self, msg):
		'''
			Parse Virtuasl Sailboat pose message - x, y, psi considered in local lambert frame R0
		'''
		self.heading_vSB = msg.pose.position.z

if __name__ == '__main__':
	rospy.init_node('camera')

	# Go to class functions that do all the heavy lifting. Do error checking.
	try:
		camera = CameraNode()
		camera.loop()
	except rospy.ROSInterruptException: pass
