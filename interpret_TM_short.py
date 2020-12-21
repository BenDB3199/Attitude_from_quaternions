import sys
from skyfield.api import load, load_file
from skyfield.constants import ERAD
from datetime import datetime, timedelta, timezone
from numpy import dot
from numpy.linalg import norm
from math import asin, acos, atan, pi
from pyquaternion import Quaternion

def parse_TM(date, time, q0, q1, q2, q3):

	dt = timedelta(milliseconds =0)		# time correction
	ts = load.timescale(builtin=True)

	planets = load_file('de435.bsp')

	satellites = load.tle_file('https://celestrak.com/satcat/tle.php?CATNR=44878')
	by_name = {sat.name: sat for sat in satellites}
	opssat = by_name['OPS-SAT']
	earth = planets['earth']
	R = ERAD 			# Earth radius in m
	sun = planets['sun']

	y = int(date[0:4])
	m = int(date[5:7])
	d = int(date[8:10])
	H = int(time[0:2])
	M = int(time[3:5])
	S = int(time[6:8])

	t_raw = datetime(y, m, d, H, M, S, tzinfo=timezone.utc) - dt
	t_sf = ts.utc(t_raw)

	timestamp = date + ' ' + time

	# flags
	earth_pointing_flag = 0

	if not (q0 == 0 and q1 ==0 and q2 == 0 and q3 == 0):

		### process quaternions ###

		q  = Quaternion(q0,q1,q2,q3)		# pyquaternion uses the convention (scalar, [vector]). Same as OBSW.
		q_ = q.inverse						# Attitude quaternion in TM is defined as transformation from ECI to S/C body frame (for both cADCS, and iADCS sensor TM)

		Xaxis = Quaternion(0,1,0,0)			# quaternions representing the S/C body frame axes
		Yaxis = Quaternion(0,0,1,0)
		Zaxis = Quaternion(0,0,0,1)

		qECI = q * Xaxis * q_					# transformation from body frame to ECI
		ECIx = [qECI[-3], qECI[-2], qECI[-1]]	# X axis
		ECIx = ECIx/norm(ECIx)

		qECI = q * Yaxis * q_
		ECIy = [qECI[-3], qECI[-2], qECI[-1]]	# Y axis
		ECIy = ECIy/norm(ECIy)

		qECI = q * Zaxis * q_
		ECIz = [qECI[-3], qECI[-2], qECI[-1]]	# Z axis
		ECIz = ECIz/norm(ECIz)

		#now we have the S/C body axes x, y, z expressed in ECI coordinates -> ECIx, ECIy, ECIz

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

		# satellite coordinates in m
		gcrs = opssat.at(t_sf)

		X = gcrs.position.km * 1000

		# nadir direction
		n = -X/norm(X)

		# calculate deviation angles
		# total deviation between -Z and nadir
		z = ECIz/norm(ECIz)
		delta_total = acos(dot(-z, n))

		if delta_total > 0.5*pi:
			earth_pointing_flag = 1 					# not pointing to earth

		delta_total = 180.0 /pi * acos(dot(-z, n))

	sys.stdout.write(timestamp + "," + str(roll)+ "," + str(pitch)  + "," + str(yaw)  + "," + str(delta_total)+ "," + str(earth_pointing_flag)+"\n" )

if __name__ == "__main__":
	date = sys.argv[1]
	time = sys.argv[2]
	qw = float(sys.argv[3])
	qx = float(sys.argv[4])
	qy = float(sys.argv[5])
	qz = float(sys.argv[6])

	parse_TM(date, time, qw, qx, qy, qz)