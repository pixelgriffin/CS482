# kf_ros.py updated file for robot estimation assignment using c++
# pset[3]
# (C) 2017 David Feil-Seifer

from kalman.srv import *

import rospy
import numpy as np
import math
from scipy import stats
import scipy.stats

# kf_update: update state estimate [u, sigma] with new control [xdot] and measurement [z]
# 	parameters:
#			u : 2x1 vector with state estimate (x) at time t-1 and control (xdot) at time t
#			sigma: 2x2 matrix with covariance at time t-1
#			z (int): observed (uncertain) measurement of state (x) at time t
#	returns: [u sigma] updated state with estimate at time t

def kf_update(u, sigma, z):
	rospy.wait_for_service('kalman_update')
	s = [0] * 4
	s[0] = sigma.item((0,0))
	s[1] = sigma.item((0,1))
	s[2] = sigma.item((1,0))
	s[3] = sigma.item((1,1))
	try:
		kalman_update = rospy.ServiceProxy('kalman_update', KalmanUpdate)
		res = kalman_update(u, s, z)
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

	sigma.itemset((0,0), res.sigma[0])
	sigma[0,0] = res.sigma[0]
	sigma[0,1] = res.sigma[1]
	sigma[1,0] = res.sigma[2]
	sigma[1,1] = res.sigma[3]

	u[0] = res.u[0]
	u[1] = res.u[1]
	return [u, sigma]


# door_update: update estimate of door locations
# 	parameters:
#			u : 2x1 vector with state estimate (x) at time t-1 and control (xdot) at time t-1
#			sigma: 2x2 matrix with covariance at time t-1
#			d (binary): door sensor at time t-1 
#			door_dist (array of size 10): probability (0..1) that a door exists at each location (0..9)
#	returns: [door_dist] updated door distribution

def door_update(u, sigma, d, door_dist):
	rospy.wait_for_service('door_update')
	s = [0] * 4
	s[0] = sigma.item((0,0))
	s[1] = sigma.item((0,1))
	s[2] = sigma.item((1,0))
	s[3] = sigma.item((1,1))
	try:
		door_update = rospy.ServiceProxy('door_update', DoorUpdate)
		res = door_update(u, s, d, door_dist)
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e
	return res.door_dist