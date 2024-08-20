#!/usr/bin/env python3

import rospy

import numpy as np
import math
from pyproj import Proj, transform
from pyproj import Transformer

from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped, Point, Twist

from bboat_pkg.msg import cmd_msg, mode_msg
from bboat_pkg.srv import mode_serv, mode_servResponse, current_target_serv, gain_serv, gain_servResponse, path_description_serv

from lib.bboat_lib import *

from command_lib import *
import time



class ControllerNode(): 
	'''
		Controller Node
		Subscribers
			- Robot pose + heading - x, y, psi
			- Virtual Sailboat position and speed 
		Publishers
			- Command - frwd speed, turning speed
		Service Clients
			- Mode, AUTO vs MANUAL
	'''
	def __init__(self): 

		# ---
		self.u_SB_thresh = 0.08
		self.dT = 0.02
		self.rate = rospy.Rate(1/self.dT)

		self.pose_robot = np.zeros((3,1))

		self.pose_vsb = np.zeros((3,1))

		self.speed_vsb = np.zeros((3,1))


		# --- Subs

		self.mode_msg = None
		self.sub_mode = rospy.Subscriber('/modepub', mode_msg, self.Mode_callback)
		rospy.wait_for_message('/modepub', mode_msg,  timeout=None)

		rospy.logwarn("HERE")

		self.sub_pose_robot = rospy.Subscriber('/pose_robot_R0', PoseStamped, self.Pose_Robot_callback)
		rospy.wait_for_message('/pose_robot_R0', PoseStamped, timeout=None)

		self.sub_vsb_pose = rospy.Subscriber('/vSBPosition', PoseStamped, self.Pose_vSB_callback)

		self.sub_vsb_speed = rospy.Subscriber('/vSBSpeed', PoseStamped, self.Speed_vSB_callback)

		self.vel_robot_RB = np.zeros((3,1))
		self.sub_vel_robot = rospy.Subscriber('/vel_robot_RB', Twist, self.Vel_Robot_callback)



		self.inte_1 = 0

		# --- Pubs
		self.pub_target = rospy.Publisher('/control_target', Point, queue_size=10)

		self.pub_cmd = rospy.Publisher('/command', cmd_msg, queue_size=10)

		# --- Services
		rospy.wait_for_service('/mode')
		connected = False
		while not connected:
			try:
				self.client_mode = rospy.ServiceProxy('/mode', mode_serv)
				connected = True
			except rospy.ServiceException as exc:
				rospy.logwarn(f'[CONTROLLER] Mode service cannot be reached - {str(exc)}')
				connected = False



		self.client_gains = rospy.ServiceProxy('/gains', gain_serv)
		# rospy.wait_for_service('/gains')
		# connected = False
		# while not connected:
		# 	try:
		# 		self.client_mode = rospy.ServiceProxy('/gains', gain_serv)
		# 		connected = True
		# 	except rospy.ServiceException as exc:
		# 		rospy.logwarn(f'[CONTROLLER] Gain service cannot be reached - {str(exc)}')
		# 		connected = False

		# rospy.wait_for_service('/current_target')
		# connected = False
		# while not connected:
		# 	try:
		# 		self.client_target = rospy.ServiceProxy('/current_target', current_target_serv)
		# 		connected = True
		# 	except rospy.ServiceException as exc:
		# 		rospy.logwarn(f'[CONTROLLER] Current Target service cannot be reached - {str(exc)}')
		# 		connected = False

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


		self.control_target = np.zeros((3,1))

		# Control variables

		self.time = 0
		# self.path_points = get_path_points()
		self.state_error_integral = 0
		self.error_state = initiate_error_state(self.pose_robot, self.vel_robot_RB, 0, self.path_points)
		self.last_u1, self.last_u2 = 0, 0

		self.s = 0


		# --- Init done
		rospy.loginfo('[CONTROLLER] Controller node Start')

	def loop(self): 

		while not rospy.is_shutdown():

			start_time = time.perf_counter()

			# mode = self.client_mode(True)
			mode = self.mode_msg

			time_subscribers1 = time.perf_counter()

			# gains = self.client_gains(True)

			time_subscribers2 = time.perf_counter()


			if mode.mode == "AUTO":	
				x_rob, y_rob, psi_rob = self.pose_robot.flatten()
				x_SB, y_SB, psi_SB = self.pose_vsb.flatten()
				#print(f'xr {x_rob} yr {y_rob} xsb {x_SB} ysb {y_SB}')
				dx_SB, dy_SB, r_SB = self.speed_vsb.flatten() 
				u1 = 0.0
				u2 = 0.0

				if mode.mission == "VSB" :
					# ---
					# u1 - Forward speed
					# kp = 1.5
					# kd = 0
					# ki = 0
					kp = gains.kp_1.data
					ki = gains.ki_1.data
					kd = gains.kd_1.data
					# print(f'inte{self.inte_1}')
					# Error in robot frame
					err_x_in_Rrob = (x_SB-x_rob)*math.cos(psi_rob) + (y_SB-y_rob)*math.sin(psi_rob)
					err_y_in_Rrob = -(x_SB-x_rob)*math.sin(psi_rob) + (y_SB-y_rob)*math.cos(psi_rob)

					# print(f'err {err_x_in_Rrob}')
					uSB_RB = dx_SB*math.cos(psi_rob) + dy_SB*math.sin(psi_rob)
					# u1 = (uSB_RB + kp*err_x_in_Rrob)
					
					# ---
					# u2 - Turning speed
					Kp = gains.kp_2.data
					u_SB =  dx_SB*math.cos(psi_SB) + dy_SB*math.sin(psi_SB)
					v_SB = -dx_SB*math.sin(psi_SB) + dy_SB*math.cos(psi_SB)

					if sqrt(err_x_in_Rrob**2+err_y_in_Rrob**2) > 3:
						print("1")
						self.inte_1 = self.inte_1 + err_x_in_Rrob*self.dT

						u1 = kd*(u_SB - self.vel_robot_RB[0]) + kp*err_x_in_Rrob + ki*self.inte_1
						# u1 = uSB_RB

						psi_des = math.atan2((y_SB-y_rob), (x_SB-x_rob))
						if psi_des <0:
							psi_des = psi_des + 2*pi

					elif u_SB > self.u_SB_thresh: 
						print("2")
						u1 = u_SB#uSB_RB

						psi_speed = math.atan2(v_SB, u_SB)
						if psi_speed <0:
							psi_speed = psi_speed + 2*pi
						psi_des = psi_SB + psi_speed

					else: 
						print("3")
						u1 = 0
						psi_des = psi_SB

					#print(f'psi_des {psi_des} r_sb {r_SB} psi_rob {psi_rob}')
					# u1 = 0
					u2 = r_SB + Kp*sawtooth(psi_des - psi_rob)

				elif mode.mission == "CAP":
					u1 = 0.0

					psi_des = 0# math.pi/2
					Kp = 0.5
					u2 = Kp*sawtooth(psi_des - psi_rob)
					
					# rospy.loginfo(f'[CONTROLLER] CAP desired = {psi_des} robot = {psi_rob}')
					# print(f'des {psi_des*180/math.pi} rob {psi_rob*180/math.pi} u2 {u2}')


				elif mode.mission == "PTN":

					command_type = "SLID" #Select command law

					# For Matrix H control (Trajectory Following):
					if(command_type == "H"):

						start_time_traj = time.perf_counter()
						target, d_target, dd_target, self.s = get_target_traj(self.s, self.dT, self.path_points)

						self.control_target = target

						start_time_com = time.perf_counter()
						u1, u2, self.state_error_integral = command_h(self.pose_robot, target, d_target, self.state_error_integral, self.dT)

						end_time_com = time.perf_counter()

					# For Path Following AUV
					if(command_type == "AUV"):

						# vitesse = np.array([[self.last_u1],[0],[self.last_u2]]) #seulement pour mode simulation
						vitesse = self.vel_robot_RB

						u_target = 1

						u1, u2, ds, xs, ys= command_auv_model(vitesse, self.error_state, u_target, self.path_points)						

						self.control_target = np.array([[xs],[ys],[0]])

						self.error_state = update_error_state(self.error_state, vitesse, self.pose_robot, ds, self.dT, self.path_points)

					# For simple LOS
					if(command_type == "LOS"):

						u_target = 1

						target = get_target_los(self.pose_robot, self.path_points)
						u1, u2, self.state_error_integral = command_los(self.pose_robot, target, u_target,self.state_error_integral, self.dT)

						trg = Point()
						trg.x = target[0]
						trg.y = target[1]
						trg.z = 0

						self.control_target = np.array([target[0][0], target[1][0], 0])

					if(command_type == "FBLIN" or command_type == "SLID"):
						
						# vitesse = np.array([[self.last_u1],[0],[self.last_u2]]) #seulement pour mode simulation
						vitesse = self.vel_robot_RB
						start_time_traj = time.perf_counter()

						target, d_target, dd_target, self.s = get_target_traj(self.s, self.dT, self.path_points)
						self.control_target = np.array([target[0][0], target[1][0], 0])						# print(self.s)
						start_time_com = time.perf_counter()

						target = target[:2]
						d_target = d_target[:2]
						dd_target = dd_target[:2] 
						dn_state = (J(self.pose_robot) @ vitesse)[:2]

						self.state_error_integral += (target - self.pose_robot[:2]) * self.dT

						if(command_type == "FBLIN"):
							k = dd_target + 0 * (d_target - dn_state) + 0.5*(target - self.pose_robot[:2]) + 0 * self.state_error_integral
						elif(command_type == "SLID"):
							k = 1*np.sign((d_target - dn_state) + 2*(target - self.pose_robot[:2]))

						u1, u2 = command_fblin(self.pose_robot, vitesse, k, self.dT)

						# print(self.last_u1)
						end_time_com = time.perf_counter()

						# self.control_target = target

				start_after_com = time.perf_counter()

				self.time += self.dT
				aux = u1
				u1 = u2
				u2 = aux
				# ---
				# Build and publish command message
				if abs(u1) > MAX_SPEED_FWRD:
					u1 = np.sign(u1)*MAX_SPEED_FWRD
				if abs(u2) > MAX_SPEED_TURN:
					u2 = np.sign(u2)*MAX_SPEED_TURN	

				# if abs(u2) > MAX_SPEED_FWRD:
				# 	u2 = np.sign(u2)*MAX_SPEED_FWRD
				# if abs(u1) > MAX_SPEED_TURN:
				# 	u1 = np.sign(u1)*MAX_SPEED_TURN	

				# Favore rotation -> don't move forward when turning is required	
				# if abs(u2) > U2_THRESH:
				# 	u1 = 0

				self.last_u1, self.last_u2 = u1, u2

				cmd = cmd_msg()
				cmd.u1 = Float64(u1)
				cmd.u2 = Float64(u2)

				self.pub_cmd.publish(cmd)

				self.pub_target.publish(Point(self.control_target[0], self.control_target[1], self.control_target[2]))


			self.rate.sleep()
			end_time = time.perf_counter()

	def Pose_Robot_callback(self, msg):
		'''
			Parse robot pose message - x, y, psi in local lambert frame R0
		'''
	
		self.pose_robot[0] = msg.pose.position.x
		self.pose_robot[1] = msg.pose.position.y
		self.pose_robot[2] = msg.pose.position.z # psi


	def Pose_vSB_callback(self, msg):
		'''
			Parse Virtuasl Sailboat pose message - x, y, psi considered in local lambert frame R0
		'''
		self.pose_vsb[0] = msg.pose.position.x
		self.pose_vsb[1] = msg.pose.position.y
		self.pose_vsb[2] = msg.pose.position.z


	def Speed_vSB_callback(self, msg):
		'''
			Parse Virtuasl Sailboat speed message - dx, dy, dpsi considered in local lambert frame R0
		'''
		self.speed_vsb[0] = msg.pose.position.x
		self.speed_vsb[1] = msg.pose.position.y
		self.speed_vsb[2] = msg.pose.position.z

	def Vel_Robot_callback(self, msg): 
		self.vel_robot_RB = np.array([[msg.linear.x], [msg.linear.y], [msg.angular.z]])

	def Current_Target_callback(self, req):
		trgt = Point()

		trgt.x, trgt.y = self.control_target[0, 0], self.control_target[1, 0]

		return trgt
	
	def Mode_callback(self, msg):
		self.mode_msg = msg


if __name__ == '__main__':
    rospy.init_node('controller')

    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        controller = ControllerNode()
        controller.loop()
    except rospy.ROSInterruptException: pass