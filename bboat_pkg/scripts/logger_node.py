#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped, Point

from time import sleep

from mavros_msgs.msg import State, OverrideRCIn

from lib.bboat_lib import *
from bboat_pkg.msg import *

import datetime, time
import os

from command_lib import *


class LoggerNode(): 
	def __init__(self):

		self.rate = rospy.Rate(100)

		self.log_flag = rospy.get_param('/bboat_logger_node/log')
		if self.log_flag:

			rospy.wait_for_service('/get_spline_points')
			connected = False
			while not connected:
				try:
					self.path_points_client = rospy.ServiceProxy('/get_spline_points', path_description_serv)
					connected = True
				except rospy.ServiceException as exc:
					rospy.logwarn(f'[CONTROLLER] Path spline service cannot be reached - {str(exc)}')
					connected = False

			self.path_points = reconstruct_spline_matrix(self.path_points_client())


			name = "first_slid"

			now = datetime.datetime.now()
			now_str = now.strftime("%Y-%m-%d_%H-%M-%S") + f"-{name}"

			os.mkdir(f"/home/luiz/log_bboat2/{now_str}")

			filename_pose_rob = f"/home/luiz/log_bboat2/{now_str}/pose_rob.txt"

			# filename_pose_vsb = f"/home/luiz/log_bboat/{now_str}/pose_vsb.txt"

			filename_command = f"/home/luiz/log_bboat2/{now_str}/command.txt"

			filename_target= f"/home/luiz/log_bboat2/{now_str}/control_target.txt"

			filename_path = f"/home/luiz/log_bboat2/{now_str}/path.txt"

			filename_vitesse = f"/home/luiz/log_bboat2/{now_str}/speed.txt"


			self.f1=open(filename_pose_rob, 'w') 

			# self.f2=open(filename_pose_vsb, 'w') 

			self.f3=open(filename_command, 'w') 

			self.f4 = open(filename_target,'w')

			self.f5 = open(filename_path,'w')

			self.file_vitesse = open(filename_vitesse,'w')

			if self.log_flag:
				for i in range(len(self.path_points)):
					np.savetxt(self.f5, self.path_points[i].flatten()[None], fmt='%s', delimiter=',')


		# --- Subs
		self.sub_pose_robot = rospy.Subscriber('/pose_robot_R0', PoseStamped, self.Pose_Robot_callback)
		# rospy.wait_for_message('/pose_robot_R0', PoseStamped, timeout=None)

		# self.sub_vsb_pose = rospy.Subscriber('/vSBPosition', PoseStamped, self.Pose_vSB_callback)

		self.sub_cmd = rospy.Subscriber('/command', cmd_msg, self.Command_callback)

		self.sub_target = rospy.Subscriber('/control_target', Point, self.Control_Target_callback)

		self.sub_vel_robot = rospy.Subscriber('/vel_robot_RB', Twist, self.Vel_Robot_callback)

		# time
		# Pose robot
		# Pose vSB
		# command

		# --- Init done
		rospy.loginfo('[LOGGER] Logger Node Start')

	def loop (self): 
		i=0
		while not rospy.is_shutdown():
			# rospy.loginfo('[LOGGER] Logger Loop')

			self.rate.sleep()
		# --- close files on shutdown
		self.f1.close()
		self.f2.close()
		self.f3.close()
		self.f4.close()
		self.f5.close()

	def Pose_Robot_callback(self, msg):
		'''
			Parse robot pose message - x, y, psi in local lambert frame R0
		'''
		t = time.time()
		arr = np.array([t, round(msg.pose.position.x,2), round(msg.pose.position.y,2), round(msg.pose.position.z,2)])
		if self.log_flag:
			np.savetxt(self.f1, arr[None], fmt='%s', delimiter=',')


	def Pose_vSB_callback(self, msg):
		'''
			Parse Virtuasl Sailboat pose message - x, y, psi considered in local lambert frame R0
		'''
		t = time.time()
		arr = np.array([t, round(msg.pose.position.x,2), round(msg.pose.position.y,2), round(msg.pose.position.z,2)])
		if self.log_flag:
			np.savetxt(self.f2, arr[None], fmt='%s', delimiter=',')

	def Command_callback(self, msg): 
		'''
		Parse command msg
		Turn forward and turning speed to 1100 - 2000 values to override
		'''
		u1, u2 = msg.u1.data, msg.u2.data
		t = time.time()

		arr = np.array([t, round(u1,2), round(u2,2)])
		if self.log_flag:
			np.savetxt(self.f3, arr[None], fmt='%s', delimiter=',')

	def Control_Target_callback(self, msg):

		t = time.time()
		arr = np.array([t, round(msg.x,2), round(msg.y,2), round(msg.z,2)])
		if self.log_flag:
			np.savetxt(self.f4, arr[None], fmt='%s', delimiter=',')

	def Vel_Robot_callback(self, msg): 

		t = time.time()
		arr = np.array([t, round(msg.linear.x,2), round(msg.linear.y,2), round(msg.angular.z,2)])
		if self.log_flag:
			np.savetxt(self.file_vitesse, arr[None], fmt='%s', delimiter=',')

			
if __name__ == '__main__':
	rospy.init_node('logger')

	# Go to class functions that do all the heavy lifting. Do error checking.
	try:
		logger = LoggerNode()
		if logger.log_flag:
			rospy.logwarn('[LOGGER] Logging')
			logger.loop()
		else:
			rospy.logwarn('[LOGGER] No logs')
	except rospy.ROSInterruptException: pass