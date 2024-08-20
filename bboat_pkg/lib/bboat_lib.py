#!/usr/bin/env python3


from pyproj import Transformer
import numpy as np
import time
import math
from numpy import mean,pi,cos,sin,sinc,sqrt,tan,arctan,arctan2,tanh,arcsin,arccos,\
                    exp,dot,array,log,inf, eye, zeros, ones, inf,size,\
                    arange,reshape,vstack,hstack,diag,median,\
                    sign,sum,meshgrid,cross,linspace,append,round,trace,rint
from numpy.random import randn,rand,uniform
from numpy.linalg import inv, det, norm, eig,qr


from geometry_msgs.msg import PoseStamped, Point, Twist
from std_msgs.msg import Float64, Bool


from bboat_pkg.srv import *


from matplotlib.pyplot import *

from bboat_pkg.msg import *

MAX_SPEED_FWRD = 2.5 #m/s Bluerobotics website 
MAX_SPEED_TURN = 2.5 #rad/s -> ~30cm between thruster and center

U2_THRESH = 0.75


lat_standard, lon_standard = 48.4180211,-4.4721604 # Bat M ENSTA


def deg_to_Lamb (x1,y1):
	transformer=Transformer.from_crs(4326,2154,always_xy=True)
	point=[(x1,y1)]
	for pt in transformer.itransform(point):
		return pt

#----------------------------------------
def sawtooth(x):
    '''
    permet d'avoir une différence d'angle nulle entre theta = 0 et theta = 2*pi
    :param x: différence d'angle
    :return: différence comprise entre [-pi,pi[
    '''
    return (x+np.pi)%(2*np.pi)-np.pi

#----------------------------------------
def f_dubins(x,u):
	''' 
		Dubins car model function
	'''
	x, y, ψ = x.flatten()
	u1, u2 = u.flatten()

	dx = u1*math.cos(ψ)
	dy = u1*math.sin(ψ)
	dpsi = u2

	return np.array([[dx], [dy], [dpsi]])

# #----------------------------------------
# # Sailboat Model function
# def f_SB(x,u, ψ, awind, P):
#     x,u=x.flatten(),u.flatten()
#     p0,p1,p2,p3,p4,p5,p6,p7,p8,p9 = P.flatten()
#     θ=x[2]; v=x[3]; w=x[4]; δr=u[0]; δsmax=u[1];
#     w_ap = np.array([[awind*cos(ψ-θ) - v],[awind*sin(ψ-θ)]])
#     ψ_ap = angle(w_ap)
#     a_ap=norm(w_ap)
#     sigma = cos(ψ_ap) + cos(δsmax)
#     if sigma < 0 :
#         δs = pi + ψ_ap
#     else :
#         δs = -sign(sin(ψ_ap))*δsmax
#     fr = p4*v*sin(δr)
#     fs = p3*(a_ap**2)* sin(δs-ψ_ap)
#     dx=v*cos(θ) + p0*awind*cos(ψ)
#     dy=v*sin(θ) + p0*awind*sin(ψ)
#     dv=(fs*sin(δs)-fr*sin(δr)-p1*v**2)/p8
#     dw=(fs*(p5-p6*cos(δs)) - p7*fr*cos(δr) - p2*w*v)/p9
#     xdot=np.array([ [dx],[dy],[w],[dv],[dw]])
#     return xdot,δs 

#----------------------------------------
def angle(x):
    x=x.flatten()
    return arctan2(x[1],x[0])

#----------------------------------------
def sawtooth(x):
    return (x+pi)%(2*pi)-pi 

#----------------------------------------
# Quat from Euler - Copied from boat_simulator.py (helios_ros2)
def quaternion_from_euler(roll, pitch, yaw):
	ai = roll / 2.0
	aj = pitch / 2.0
	ak = yaw / 2.0
	ci = math.cos(ai)
	si = math.sin(ai)
	cj = math.cos(aj)
	sj = math.sin(aj)
	ck = math.cos(ak)
	sk = math.sin(ak)
	cc = ci*ck
	cs = ci*sk
	sc = si*ck
	ss = si*sk

	q = np.empty((4, ))
	q[0] = cj*sc - sj*cs
	q[1] = cj*ss + sj*cc
	q[2] = cj*cs - sj*sc
	q[3] = cj*cc + sj*ss

	return q

#----------------------------------------	
def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    
    return roll_x, pitch_y, yaw_z # in radians

#----------------------------------------
