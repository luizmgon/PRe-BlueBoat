#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import Pose, Point
from scipy.interpolate import CubicSpline

from bboat_pkg.srv import next_target_serv, next_target_servResponse, lambert_ref_serv, current_target_serv, path_description_serv, path_description_servResponse
from lib.bboat_lib import *

class MissionNode(): 
	def __init__(self): 

		self.mission_file = rospy.get_param('/bboat_mission_node/filepath')
		
		self.mission_points = self.Parse_Mission_File()

		self.mission_simu = rospy.get_param('/bboat_mission_node/mission_simu')

		self.current_target = self.mission_points[0,:]

		self.next_point_index = 0


		self.ref_lamb = np.zeros((2,1))

		# if not self.mission_simu: 
		rospy.wait_for_service('/lambert_ref')
		connected = False
		while not connected:
			try:
				self.client_ref_lambert = rospy.ServiceProxy('/lambert_ref', lambert_ref_serv)
				connected = True
			except rospy.ServiceException as exc:
				rospy.logwarn(f'[MISSION] Lambert ref service cannot be reached - {str(exc)}')
				connected = False
		resp = self.client_ref_lambert(True)
		self.ref_lamb[0,0] = resp.lambert_ref.x
		self.ref_lamb[1,0] = resp.lambert_ref.y

		self.mission_spline = self.get_path_spline_points()


		rospy.Service('/next_target', next_target_serv, self.Next_Target_callback)

		rospy.Service('/current_target', current_target_serv, self.Current_Target_callback)

		rospy.Service('/get_spline_points', path_description_serv, self.handle_get_spline_points)


		#print(f'mission : first ref lambert = {self.ref_lamb[0,0]} | {self.ref_lamb[1,0]}')

		# --- Init done
		rospy.loginfo('[MISSION] Mission Node Start')



	def loop(self): 
		while not rospy.is_shutdown():
			rospy.sleep(1)

	def Next_Target_callback(self, req): 
		# rospy.loginfo('[MISSION] Next target service callback')
		next_point = Pose()
		if self.next_point_index < len(self.mission_points): 
			rospy.loginfo('[MISSION] Next point, continue mission')
			continue_mission = True
			lat, lon = self.mission_points[self.next_point_index, 0], self.mission_points[self.next_point_index, 1]

			next_point_lambert = deg_to_Lamb(lon, lat)

			#print(f'mission : next point lambert {next_point_lambert[0]} | {next_point_lambert[1]}')
			if not self.mission_simu: 
				resp = self.client_ref_lambert(True)
				self.ref_lamb[0,0] = resp.lambert_ref.x
				self.ref_lamb[1,0] = resp.lambert_ref.y


			#print(f'mission : next point lambert apres {next_point_lambert[0]- self.ref_lamb[0,0]} | {next_point_lambert[1]- self.ref_lamb[1,0]}')

			# next_point.position.x = next_point_lambert[1]- self.ref_lamb[1,0]
			# next_point.position.y = next_point_lambert[0]- self.ref_lamb[0,0]	

			next_point.position.x = next_point_lambert[1]- self.ref_lamb[0,0]
			next_point.position.y = next_point_lambert[0]- self.ref_lamb[1,0]	


			print(f'-------------- point {self.next_point_index} lat {lat}, lon {lon} x {next_point.position.x}, y {next_point.position.y}')

			self.current_target = np.array([[next_point.position.x], [next_point.position.y]])
			self.next_point_index +=1
		else:
			rospy.loginfo('[MISSION] Last point, end mission')
			continue_mission = False

		response = next_target_servResponse()
		response.next_trgt_pose = next_point
		response.continuing_mission = continue_mission	
		return response

	def	Parse_Mission_File(self): 
		file = open(self.mission_file)
		lines = file.readlines()
		px = []
		py = []
		for line in lines: 
			tab = line.split(",")
			px.append(float(tab[0]))
			py.append(float(tab[1]))
		points = np.array([px,py])

		return points.transpose()

	def Ref_Lambert_callback(self, msg):
		self.ref_lamb[0,0] = msg.x
		self.ref_lamb[1,0] = msg.y

	def Current_Target_callback(self, req):
		trgt = Point()

		if not self.mission_simu: 
			lat, lon = self.mission_points[0, 0], self.mission_points[0, 1]
			next_point_lambert = deg_to_Lamb(lon, lat)

			#print(f'mission : next point lambert {next_point_lambert[0]} | {next_point_lambert[1]}')

			resp = self.client_ref_lambert(True)
			self.ref_lamb[0,0] = resp.lambert_ref.x
			self.ref_lamb[1,0] = resp.lambert_ref.y


			#print(f'mission : next point lambert apres {next_point_lambert[0]- self.ref_lamb[0,0]} | {next_point_lambert[1]- self.ref_lamb[1,0]}')

			trgt.x, trgt.y = next_point_lambert[1]- self.ref_lamb[0,0], next_point_lambert[0]- self.ref_lamb[1,0]
		else: 
			trgt.x, trgt.y = self.mission_points[0, 0], self.mission_points[0, 1]
		return trgt
	

	def get_path_spline_points(self):

		x = []
		y = []

		lat, lon = self.mission_points[:, 0], self.mission_points[:, 1]
		resp = self.client_ref_lambert(True)
		self.ref_lamb[0,0] = resp.lambert_ref.x
		self.ref_lamb[1,0] = resp.lambert_ref.y

		for i in range(len(lat)):
			next_point_lambert = deg_to_Lamb(lon[i], lat[i])
			x.append(next_point_lambert[1]- self.ref_lamb[0,0])
			y.append(next_point_lambert[0]- self.ref_lamb[1,0])

		# Initial x and y coordinates
		# x = np.array([2, -3, -8, -3, 2, -3, -8])

		print(x)
		print(y)


		# Cubic spline interpolation
		cs = CubicSpline(x, y)

		# Generate 10,000 evenly spaced points and interpolate x
		x = np.linspace(x[0], x[-1], 1000)
		y = cs(x)

		# Calculate distances and arc length
		distances = np.sqrt(np.diff(x)**2 + np.diff(y)**2)
		s = np.zeros_like(x)
		s[1:] = np.cumsum(distances)
		ds = np.gradient(s)

		# Calculate derivatives
		dx = np.gradient(x) / ds
		dy = np.gradient(y) / ds
		ddx = np.gradient(dx) / ds
		ddy = np.gradient(dy) / ds

		# Calculate orientation angle and curvature
		phi_f = np.arctan2(dy, dx)
		curvature = (dx * ddy - dy * ddx) / (dx**2 + dy**2)**1.5
		dc = np.gradient(curvature)
		g_c = dc / ds

		# Return results
		return np.vstack((x, y, s, phi_f, curvature, g_c, dx, dy, ddx, ddy))
	
	def handle_get_spline_points(self, req):
		result = self.mission_spline
		return path_description_servResponse(x=result[0], y=result[1], s=result[2], phi_f=result[3], curvature=result[4], g_c=result[5], dx=result[6], dy=result[7], ddx=result[8], ddy=result[9])




if __name__ == '__main__':
	rospy.init_node('mission')

	# Go to class functions that do all the heavy lifting. Do error checking.
	try:
		mission = MissionNode()
		mission.loop()
	except rospy.ROSInterruptException: pass
