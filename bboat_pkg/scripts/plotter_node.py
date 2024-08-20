#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import Float64

from time import sleep
import time

from mavros_msgs.msg import State, OverrideRCIn

from lib.bboat_lib import *
from bboat_pkg.msg import *
from bboat_pkg.srv import path_description_serv


from command_lib import *

import datetime, time
import os

class PlotterNode(): 
	def __init__(self):

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

		self.rate = rospy.Rate(15)

		self.flag_plotting = rospy.get_param('/bboat_plotter_node/plot')

		self.mode_simu = rospy.get_param('/bboat_plotter_node/mission_simu')



		# --- Subs
		self.sub_pose_robot = rospy.Subscriber('/pose_robot_R0', PoseStamped, self.Pose_Robot_callback)
		self.sub_target = rospy.Subscriber('/control_target', Point, self.Control_Target_callback)
		# rospy.wait_for_message('/pose_robot_R0', PoseStamped, timeout=None)
		self.x_rob_store = []
		self.y_rob_store = []
		self.psi_rob_store = []
		self.pose_rob = np.zeros((3,1))



		# rospy.wait_for_service('/current_target')
		# self.client_target = rospy.ServiceProxy('/current_target', current_target_serv)

		# rospy.wait_for_service('/control_target')
		# self.control_target = rospy.ServiceProxy('/control_target', current_target_serv)
		self.control_target = Point()

		if not self.mode_simu: 
			self.sub_vsb_pose = rospy.Subscriber('/vSBPosition', PoseStamped, self.Pose_vSB_callback)
			self.x_vsb_store = []
			self.y_vsb_store = []
			self.psi_vsb_store = []
			self.pose_vsb = np.zeros((3,1))
			self.vel_vsb_Rvsb = np.zeros((3,1))
			self.u_vsb_store = []
			self.v_vsb_store = []
			self.r_vsb_store = []
			self.sub_vsb_speed = rospy.Subscriber('/vSBSpeed', PoseStamped, self.Speed_vSB_callback)

			self.sub_a = rospy.Subscriber('/a', Point, self.a_callback)
			self.a = np.zeros((2,1))
			self.sub_b = rospy.Subscriber('/b', Point, self.b_callback)
			self.b = np.zeros((2,1))

		self.vel_robot_RB = np.zeros((3,1))
		self.u_rob_store = []
		self.v_rob_store = []
		self.r_rob_store = []
		self.sub_vel_robot = rospy.Subscriber('/vel_robot_RB', Twist, self.Vel_Robot_callback)


		self.u1, self.u2 = 0, 0
		self.u1_store = []
		self.u2_store = []
		self.sub_cmd = rospy.Subscriber('/command', cmd_msg, self.Command_callback)
		# time
		# Pose robot
		# Pose vSB
		# command

		# --- Matplotlib plotting

		# if self.flag_plotting:
		# 	self.fig = figure(1)
		# 	self.plot_wind = self.fig.add_subplot(111, aspect='equal')
		# 	if not self.mode_simu: 
		# 		self.fig2 = figure(2)
		# 		self.plot_wind2 = self.fig2.add_subplot(131, aspect='equal')


		# --- Init done
		rospy.loginfo('[PLOTTER] Plotter Node Start')

	def loop (self): 
		i=0
		while not rospy.is_shutdown():
			# rospy.loginfo('[PLOTTER] Plotter Loop')
			start_time = time.perf_counter()


			# self.x_rob_store.append(self.pose_rob[0,0])
			# self.y_rob_store.append(self.pose_rob[1,0])
			# self.psi_rob_store.append(self.pose_rob[2,0])

			# self.u1_store.append(self.u1)
			# self.u2_store.append(self.u2)

			# self.u_rob_store.append(self.vel_robot_RB[0,0])
			# self.v_rob_store.append(self.vel_robot_RB[1,0])
			# self.r_rob_store.append(self.vel_robot_RB[2,0])

			# if not self.mode_simu: 
			# 	self.u_vsb_store.append(self.vel_vsb_Rvsb[0,0])
			# 	self.v_vsb_store.append(self.vel_vsb_Rvsb[1,0])
			# 	self.r_vsb_store.append(self.vel_vsb_Rvsb[2,0])
				
			# 	self.x_vsb_store.append(self.pose_vsb[0,0])
			# 	self.y_vsb_store.append(self.pose_vsb[1,0])
			# 	self.psi_vsb_store.append(self.pose_vsb[2,0])

			self.Plot_1()

			# self.Plot_2()

			self.rate.sleep()
			# end_time = time.perf_counter()
			# print(f"{int(1/(end_time - start_time))} ms")


	def Pose_Robot_callback(self, msg):
		'''
			Parse robot pose message - x, y, psi in local lambert frame R0
		'''
		self.pose_rob = np.array([[msg.pose.position.x], [msg.pose.position.y], [msg.pose.position.z]])

	
	def Pose_vSB_callback(self, msg):
		'''
			Parse Virtuasl Sailboat pose message - x, y, psi considered in local lambert frame R0
		'''
		self.pose_vsb = np.array([[msg.pose.position.x], [msg.pose.position.y], [msg.pose.position.z]])

		

	def Command_callback(self, msg): 
		'''
		Parse command msg
		Turn forward and turning speed to 1100 - 2000 values to override
		'''
		self.u1, self.u2 = msg.u1.data, msg.u2.data

			

	def Speed_vSB_callback(self, msg):
		'''
			Parse Virtuasl Sailboat speed message - dx, dy, dpsi considered in local lambert frame R0
		'''

		self.vel_vsb_Rvsb = np.array([[msg.pose.position.x], [msg.pose.position.y], [msg.pose.position.z]])

	def Control_Target_callback(self, msg):

		self.control_target = msg


	def Vel_Robot_callback(self, msg): 
		self.vel_robot_RB = np.array([[msg.linear.x], [msg.linear.y], [-msg.angular.z]])


	def a_callback(self, msg):
		self.a[0,0] = msg.x
		self.a[1,0] = msg.y

	def b_callback(self, msg):
		self.b[0,0] = msg.x
		self.b[1,0] = msg.y



	def Plot_1(self): 
		# Plot en y abscisse, x ordonnée pour avoir North vers le haut
		figure(1)
		cla()
		# self.plot_wind.grid()
		# self.plot_wind.invert_xaxis()
		xlabel('y_0 : East')
		ylabel('x_0 : North')


		if not self.mode_simu: 
			# plot([self.a[1,0], self.b[1,0]], [self.a[0,0], self.b[0,0]], 'k')

			# plot(self.y_vsb_store, self.x_vsb_store, '--r')
			# plot(self.pose_vsb[1,0], self.pose_vsb[0,0], 'or')
			# plot([self.pose_vsb[1,0], self.pose_vsb[1,0]+5*sin(self.pose_vsb[2,0])], [self.pose_vsb[0,0], self.pose_vsb[0,0]+5*cos(self.pose_vsb[2,0])], 'r')
			pt = self.control_target
			# print(pt)
			plot( pt.y, pt.x, 'ok')
		else: 
			pt = self.control_target
			# print(pt)
			plot( pt.y, pt.x, 'ok')

		x_rob, y_rob, psi_rob = self.pose_rob.flatten()

		plot(y_rob, x_rob, 'ob')
		plot([y_rob, y_rob+5*sin(psi_rob)], [x_rob, x_rob+5*cos(psi_rob)], 'b')

		x_values, y_values = self.path_points[:2]
		plot(y_values, x_values, color="green", linestyle="--")

		x_limit = [min(min(y_values),0) - 50, max(max(y_values),0) + 50]
		y_limit = [min(min(x_values),0) - 50, max(max(x_values),0) + 50]

		xlim(x_limit)
		ylim(y_limit)

		# plot([self.pose_vsb[1,0], self.pose_vsb[1,0]+5*sin(self.θ_bar)],  [self.pose_vsb[0,0], self.pose_vsb[0,0]+5*cos(self.θ_bar)], 'g')

		# wind = 10*self.awind * np.array([[cos(self.ψ)], [sin(self.ψ)]])
		# quiver(self.b[1,0]+3, self.b[0,0]+3, wind[1,0], wind[0,0])

		pause(.0001)
		show(block=False)

	def Plot_2(self): 
		figure(2)
		subplot(3,1,1)
		cla()
		grid()
		ylabel('u')
		if len(self.u_vsb_store) > 1000: 
			plot(self.u_vsb_store[len(self.u_vsb_store)-1000:len(self.u_vsb_store)-1], '-r')
			plot(self.u_rob_store[len(self.u_vsb_store)-1000:len(self.u_vsb_store)-1], '-b')
			plot(self.u1_store[len(self.u_vsb_store)-1000:len(self.u_vsb_store)-1], '-g')
		else: 
			plot(self.u_vsb_store, '-r')
			plot(self.u_rob_store, '-b')
			plot(self.u1_store, '-g')

		subplot(3,1,2)
		cla()
		grid()
		if len(self.u_vsb_store) > 1000: 
			plot(self.v_vsb_store[len(self.v_vsb_store)-1000:len(self.v_vsb_store)-1], '-r')
			plot(self.v_rob_store[len(self.v_vsb_store)-1000:len(self.v_vsb_store)-1], '-b')
		else: 
			plot(self.v_vsb_store, '-r')
			plot(self.v_rob_store, '-b')
		ylabel('v')


		subplot(3,1,3)
		cla()
		grid()
		if len(self.u_vsb_store) > 1000: 
			plot(self.r_vsb_store[len(self.r_vsb_store)-1000:len(self.r_vsb_store)-1], '-r')
			plot(self.r_rob_store[len(self.r_vsb_store)-1000:len(self.r_vsb_store)-1], '-b')
			plot(self.u2_store[len(self.r_vsb_store)-1000:len(self.r_vsb_store)-1], '-g')
		else: 
			plot(self.r_vsb_store, '-r')
			plot(self.r_rob_store, '-b')
			plot(self.u2_store, '-g')
		ylabel('r')

		pause(0.001)
		show(block=False)


if __name__ == '__main__':
	rospy.init_node('plotter')

	# Go to class functions that do all the heavy lifting. Do error checking.
	try:
		plotter = PlotterNode()
		if plotter.flag_plotting:
			plotter.loop()
	except rospy.ROSInterruptException: pass