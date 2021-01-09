#!/usr/bin/env python3

import sys
import ephem
from datetime import datetime, timezone
from numpy import dot
from numpy.linalg import norm
from math import asin, acos, atan, pi, cos, sin
from pyquaternion import Quaternion

class TLE:
	def __init__(self, line1, line2, line3):
		""" Creates and object containing the lines of an TLE.

		Parameters
		----------
		line1 : str
			first line of the TLE
		line2 : str
			second line of the TLE
		line3 : str
			third line of the TLE
		"""
		self.line1 = line1
		self.line2 = line2
		self.line3 = line3

class satpos:
	def __init__(self, ra, dec):
		""" Computes the cartesian coordinates from equatorial coordinates assuming radius of 1 (normed).

		Parameters
		----------
		ra : float
			right inclanation
		dec : float
			declination
		"""
		self.x = cos(dec) * cos(ra)
		self.y = cos(dec) * sin(ra)
		self.z = sin(dec)

def quaternions2euler(timestamp, q0, q1, q2, q3, TLE):
	""" Claculates the euler angles from quaternions and writes them to the standard out.

	Parameters
	----------
	date
	time
	q0
	q1
	q2
	q3
	TLE

	Returns
	-------
	all return values are written to the standard out port.
	"""

	OPS = ephem.readtle(TLE.line1, TLE.line2, TLE.line3)

	# flags
	earth_pointing_flag = 0

	if not (q0 == 0 and q1 ==0 and q2 == 0 and q3 == 0):

		### process quaternions ###
		# pyquaternion uses the convention (scalar, [vector]). Same as OBSW.
		q  = Quaternion(q0,q1,q2,q3)
		q_ = q.inverse

		# quaternions representing the S/C body frame axes
		Zaxis = Quaternion(0,0,0,1)

		qECI = q * Zaxis * q_
		ECIz = [qECI[-3], qECI[-2], qECI[-1]]	# Z axis
		ECIz = ECIz/norm(ECIz)

		#now we have the S/C body axes z expressed in ECI coordinates -> ECIz

		# calculating euler angles roll => x, pitch => y, yaw => z
		# roll
		roll = 2*(q0 * q1 + q2 * q3)
		roll = roll/(1-2*(q1**2 + q2**2))
		roll = 180.0 /pi * atan(roll)
		if abs(q1) >= 0.71:
			roll = 180 + roll

		# pitch
		pitch = 2*(q0 * q2 - q3 * q1)
		pitch = 180.0 /pi * asin(pitch)
		if abs(q2) >= 0.71:
			pitch = 180 - pitch

		# yaw
		yaw = 2*(q0 * q3 + q1 * q2)
		yaw = yaw/(1-2*(q2**2 + q3**2))
		yaw = 180.0 /pi * atan(yaw)
		if abs(q3) >= 0.71:
			yaw = 180 + yaw

		# calculate nadir direction
		OPS.compute(timestamp)
		OPSsat = satpos(float(OPS.ra), float(OPS.dec))
		OPSsat = [OPSsat.x, OPSsat.y, OPSsat.z]

		n = -1 * (OPSsat/norm(OPSsat))

		# total deviation angle between -Z and nadir
		z = ECIz/norm(ECIz)
		delta_total = acos(dot(-z, n))

		if delta_total > 0.5*pi:
			# 1 = not pointing to earth
			earth_pointing_flag = 1

		delta_total = 180.0 /pi * acos(dot(-z, n))

	sys.stdout.write(str(timestamp) + "," + str(roll)+ "," + str(pitch)  + "," + str(yaw)  + "," + str(delta_total)+ "," + str(earth_pointing_flag)+"\n" )

if __name__ == "__main__":
	f_q = open('quaternion.txt', 'r')
	f_q_line = f_q.read()
	f_q_line = f_q_line.split()

	date_time_str = f_q_line[0] + ' ' + f_q_line[1]
	print(date_time_str)
	date_time_obj = datetime.strptime(date_time_str, '%Y-%m-%d %H:%M:%S.%f')
	qw = float(f_q_line[2])
	qx = float(f_q_line[3])
	qy = float(f_q_line[4])
	qz = float(f_q_line[5])

	f_tle = open('tle.txt', 'r')
	f_tle_lines = f_tle.readlines()
	tle = []
	for line in f_tle_lines:
		tle.append(line)

	TLE_cur = TLE(tle[0], tle[1], tle[2])

	quaternions2euler(date_time_obj, qw, qx, qy, qz, TLE_cur)
