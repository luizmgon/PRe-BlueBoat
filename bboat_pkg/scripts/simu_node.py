#!/usr/bin/env python3

import rospy
import numpy as np
import math

from geometry_msgs.msg import  PoseStamped, TwistStamped, Point
from bboat_pkg.msg import mode_msg
# from bboat_pkg.srv import mode_serv, mode_servResponse
# from bboat_pkg.msg import cmd_msg
from lib.bboat_lib import *

class SimuNode():
	'''
		Simulation node usefull for command testing
		Replaces sensor and driver nodes
		Subscribers
			- Command 
		Publishers
			- Robot Pose in local frame - x, y, psi
		Service Provider
			- Mode - Always on AUTO in simulation
	'''
	def __init__(self):
		# --- Simulation parameters
		self.dT = 0.05
		self.rate = rospy.Rate(1/self.dT)

		# --- Simulated robot state and cmd
		self.X = np.array([[0], [0], [0*np.pi]])
		self.U = np.zeros((2,1))

		# --- Subs
		self.sub_cmd = rospy.Subscriber('/command', cmd_msg, self.Command_callback)
		self.auto_cmd = np.zeros((2,1))
		self.max_speed_fwrd = 3 #m/s Bluerobotics website 
		self.max_speed_turn = 1 #rad/s -> ~30cm between thruster and center

		# --- Pubs
		self.mode_publisher = rospy.Publisher('/modepub', mode_msg , queue_size=10)

		while self.mode_publisher.get_num_connections() == 0:
			rospy.loginfo("Esperando por subscribers...")
			rospy.sleep(1)

		first_mode_msg = mode_msg()
		first_mode_msg.mode = "AUTO"
		first_mode_msg.mission = "PTN"


		self.mode_publisher.publish(first_mode_msg)
		self.pub_pose_R0 = rospy.Publisher('/pose_robot_R0', PoseStamped, queue_size=10)

	
		# --- Services
		rospy.Service('/mode', mode_serv, self.Mode_Service_callback)


		rospy.Service('/gains', gain_serv, self.Gain_Service_callback)

		# --- Init done
		rospy.loginfo('[SIMU] Simu node Start')


	def loop(self):
		while not rospy.is_shutdown():
			# --- Dubins car model integration (Euler)
			dX = f_dubins(self.X, self.auto_cmd)
			self.X = self.X + dX*self.dT

			# --- Build and publish pose message
			pose_msg = PoseStamped()
			pose_msg.pose.position.x = self.X[0]
			pose_msg.pose.position.y = self.X[1]
			pose_msg.pose.position.z = self.X[2]
			self.pub_pose_R0.publish(pose_msg)

			self.rate.sleep()


	def Command_callback(self, msg): 
		'''
			Parse command message received from control node
			cmd considered as fwrd speed and heading speed
		'''	
		u1, u2 = msg.u1.data, msg.u2.data
		# --- Limit cmds to max speeds
		if abs(u1) > self.max_speed_fwrd:
			u1 = np.sign(u1)*self.max_speed_fwrd
		if abs(u2) > self.max_speed_turn:
			u2 = np.sign(u2)*self.max_speed_turn

		fwrd = u1
		turn = u2

		self.auto_cmd = np.array([[fwrd], [turn]])

	def Mode_Service_callback(self, req):
		'''
			Mode service callback always respond with AUTO in simulation
		'''
		resp = mode_servResponse()
		resp.mode = "AUTO"
		resp.mission = "PTN"
		return resp

	def Gain_Service_callback(self, req): 
		resp = gain_servResponse()

		resp.kp_1 = Float64(1.5)

		resp.kd_1 = Float64(0)

		resp.ki_1 = Float64(0)

		resp.kp_2 = Float64(0.5)

		resp.kd_2 = Float64(0)

		resp.ki_2 = Float64(0)
		# resp = self.gains
		return resp

# Main function.
if __name__ == '__main__':
    rospy.init_node('simu')

    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        sim= SimuNode()
        sim.loop()
    except rospy.ROSInterruptException: pass