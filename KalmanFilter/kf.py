# kf.py updated file for robot estimation assignment
# pset[3]
# (C) 2017 David Feil-Seifer


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
	return [u, sigma]


# door_update: update estimate of door locations
# 	parameters:
#			u : 2x1 vector with state estimate (x) at time t-1 and control (xdot) at time t-1
#			sigma: 2x2 matrix with covariance at time t-1
#			d (binary): door sensor at time t-1 
#			door_dist (array of size 10): probability (0..1) that a door exists at each location (0..9)
#	returns: [door_dist] updated door distribution

def door_update(u, sigma, d, door_dist):
	return door_dist