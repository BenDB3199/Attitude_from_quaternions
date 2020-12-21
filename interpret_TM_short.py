from skyfield.api import EarthSatellite, Topos, load, load_file
from skyfield.constants import ERAD
import skyfield
from datetime import datetime, timedelta, timezone
from numpy import cross, dot, array, matrix, outer, matmul, einsum, clip
from numpy.linalg import norm, eig
from math import radians, degrees, sin, cos, asin, acos, sqrt, atan, pi
from pyquaternion import Quaternion



def to_deg(x):
	if x == "" :
		return ""
	else :
		return 180.0 /pi *float(x)
		
def parse_TM(filename):
	
	###########
	f_in = open('input_data\\' + filename,"r")
	f_att = open("output_data\\attitude.dat","w+")
	f_loc = open("output_data\\location.dat","w+")
	f_deb = open("output_data\\debug.dat","w+")
	dt = timedelta(milliseconds =0)		# time correction
	###########
	
	
	ts = load.timescale(builtin=True)
	
	planets = load_file('de435.bsp')

	satellites = load.tle_file('https://celestrak.com/satcat/tle.php?CATNR=44878')
	by_name = {sat.name: sat for sat in satellites}
	opssat = by_name['OPS-SAT']
	earth = planets['earth']
	R = ERAD 			# Earth radius in m
	sun = planets['sun']
	
	
	f_in_lines = f_in.readlines()
	
	header = True
	
	
	for line in f_in_lines:
	
		line = line.rstrip()
		if header == False :
	
			y = int(line[0:4])
			m = int(line[5:7])
			d = int(line[8:10])
			H = int(line[11:13])
			M = int(line[14:16])
			S = int(line[17:19])			
			
			t_raw = datetime(y, m, d, H, M, S, tzinfo=timezone.utc) - dt
			t_sf = ts.utc(t_raw)
			
			vals = line.split(",")
			timestamp = vals[0]
			line = line[19:].replace(" ","")
			vals = line.split(",")
			
			q_pd_xPlus = ""
			q_pd_xMinus = ""
			q_pd_yPlus= ""
			q_pd_yMinus = ""
			q_pd_zPlus= ""
			q_pd_zMinus = ""
			delta_fwd = ""
			delta_side = ""
			Ydev = ""
			beta = ""
			gamma = ""
			rho = ""
			sigma = ""
			sc_lat = ""
			sc_long = ""
			target_lat = ""
			target_long = ""
			q0 = ""
			q1 = ""
			q2 = ""
			q3 = ""
			
			
			# OBSW naming convention, see OBSW/src/ASW/DH/Acquisitor/ASW_DH_ACQ_Task.c:
			# PD1 -> Z+
			# PD2 -> Y-
			# PD3 -> X+
			# PD4 -> Y+
			# PD5 -> X-
			# PD6 -> Z-
			
			pd_zPlus = to_deg(vals[5])
			pd_yMinus = to_deg(vals[6])
			pd_xPlus = to_deg(vals[7])
			pd_yPlus = to_deg(vals[8])
			pd_xMinus = to_deg(vals[9])
			pd_zMinus = to_deg(vals[10])
			
			
			if vals[1] != "" and vals[2] != "" and vals[3] != "" and vals[4] != "" :
			
				q0 = float(vals[1])
				q1 = float(vals[2])
				q2 = float(vals[3])
				q3 = float(vals[4])
				if not (q0 == 0 and q1 ==0 and q2 == 0 and q3 == 0):
				
					### process quaternions ###	
				
					q  = Quaternion(q0,q1,q2,q3)		# pyquaternion uses the convention (scalar, [vector]). Same as OBSW. 
														# Attitude quaternion in TM is defined as transformation from ECI to S/C body frame (for both cADCS, and iADCS sensor TM)
					q_ = q.inverse
					
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


					### vector geometry to interpret attitude, using orbit info and position of Earth & Sun from Skyfield ###
					

					# satellite coordinates in m
					gcrs = opssat.at(t_sf)
					
					X = gcrs.position.km * 1000
					
					# nadir direction
					n = -X/norm(X)
					
					# flight direction, from velocity
					V = gcrs.velocity.km_per_s * 1000
					v = V/norm(V)		
					
					# since the orbit is not circular, we cannot expect v to be fully perpendicular to n. However, it is also useful to have a modified flight direction w that is perpendicular to n and resides in the v-n plane
					W = v - dot(v,n)*n
					w = W/norm(W)
					
					# Sun position in m
					sun_pos = earth.at(t_sf).observe(sun)
					S = sun_pos.position.km * 1000		
					
					# unit vector b from satellite to sun
					B = S - X
					b = B/norm(B)
					

					# calculate deviation angles
					# deviation between -Z and nadir, split into forward  and sideways deviation
					
					# first, project pointing vector p = -ECIz into the orbit plane (defined by vectors w, n)
					o = cross (w,n)

					p = -ECIz
					p_proj = p - dot(p, o) * o
					p_proj = p_proj/ norm(p_proj)
					
					
					# now calculate the delta between p_proj and nadir
					delta_fwd = 180.0 /pi * acos(dot(p_proj,n))
					
					# add sign: + means "nose up", i.e. -Z is pointing forward 
					if acos(dot(p_proj, w)) > 0.5*pi :
						delta_fwd = -1 * delta_fwd
						
						

					# project pointing vector into the plane defined by the forward direction w:
					p_proj = p - dot(p, w) * w
					p_proj = p_proj/ norm(p_proj)
					
					# now calculate the sideways deviation angle as delta between p_proj and nadir 
					delta_side = 180.0 /pi * acos(clip(dot(p_proj,n),-1,1))
					
					# add sign: + means right side down,  i.e. -Z is pointing to the left of the ground track
					if acos(dot(p_proj, o)) > 0.5*pi :
						delta_side = -1 * delta_side
					

					
					# we assume that we have -Y pointing in flight direction as a secondary attitude constraint -> calculate total deviation from flight direction, do not apply sign convention
					A = cross (v,ECIz)
					a = A/norm(A)
					
					p = -ECIy
					p_proj = p - dot(p, a) * a
					p_proj = p_proj/ norm(p_proj)
					
					Ydev = 180.0 /pi * acos(clip(dot(-ECIy,v),-1,1))			



					# compass direction of ground target wrt nadir vector, using the convention North = 0 deg, East = 90 deg etc.
					# north direction: let there be point Z = (0,0,z) with Z-X perpendicular to the nadir direction  => calculate z from X
					Z = [0,0,norm(X) * norm(X) / X[2] ]
					# north direction m = |Z-X|
					M = Z-X
					m = M/norm(M)
					
					if norm(X) * norm(X) / X[2] < 0 :
						# we're on the southern hemisphere, so this approach needs to be inverted
						m = -m
					
					
					# east direction e
					e = cross (n,m)
					
					# project pointing vector p = -ECIz into plane defined by nadir vector n
					p = -ECIz
					p_proj = p - dot(p, n) * n
					p_proj = p_proj/ norm(p_proj)
					
					
					# deviation from North direction
					gamma = 180.0 /pi * acos(clip(dot(p_proj, m),-1,1))

					
					# now use East direction to apply correct sign and range convention
					if acos(dot(p_proj,e)) > 0.5*pi :	# 2nd or 3rd quadrant
						gamma = 360 - gamma
					

					# camera orientation within spacecraft: image coordinate system is +X, +Y in body frame 
					# => if -Y is pointing south, the image will correspond with the map orientation. -Y is supposed to point in flight direction, so we expect only small rotations
					# sign convention for rotation angle: + = anticlockwise
					
					# project +Y direction into plane perpendicular to nadir vector n
					p = ECIy
					p_proj = p - dot(p, n) * n
					p_proj = p_proj/ norm(p_proj)
					
					# deviation from North direction
					rho = 180.0 /pi * acos(clip(dot(p_proj, m),-1,1))
					
					# now use East direction to apply correct sign 
					if acos(dot(p_proj,e)) < 0.5*pi :	
						rho = - rho

					# deviation between -X and sun direction
					beta = 180.0 /pi * acos(clip(dot(b,-ECIx),-1,1))
					
					
					# target lat/long
					# rough calculation of target latitude/longitude assuming Earth is a sphere of radius R
					# this should probably be refined using an oblate instead of spherical Earth -> see https://www.celestrak.com/columns/v02n03/ for thoughts on accuracy
					subpoint = gcrs.subpoint()
					sc_lat = subpoint.latitude.degrees
					sc_long = subpoint.longitude.degrees
					
					# change of latitude: project pointing vector p = -ECIz into plane defined by nadir and north
					a = cross (n,m)
					p = -ECIz
					p_proj = p - dot(p, a) * a
					p_proj = p_proj/ norm(p_proj)
					
					target_lat = "N/A"
					if abs(norm(X)/R * sin(acos(dot(p_proj,n)))) <= 1:
						delta_lat = 180.0 /pi * ( asin(norm(X)/R * sin(acos(dot(p_proj,n)))) - acos(dot(p_proj,n)))
						
						# sign convention: latitude increases towards the north
						if acos(dot(p_proj,n)) > 0.5*pi :	
							delta_lat = - delta_lat
							
						target_lat = sc_lat + delta_lat
						
						if target_lat < -180:
							target_lat += 360
						if target_lat > 180:
							target_lat -= 360
							

					# change of longitude: project pointing vector p = -ECIz into plane defined by nadir and east
					a = cross (n,e)
					p = -ECIz
					p_proj = p - dot(p, a) * a
					p_proj = p_proj/ norm(p_proj)
					
					target_long = "N/A"
					if abs(norm(X)/R * sin(acos(dot(p_proj,n)))) <= 1:
						delta_long = 180.0 /pi * ( asin(norm(X)/R * sin(acos(dot(p_proj,n)))) - acos(dot(p_proj,n)))
						# sign convention: longitude increases towards the east
						if acos(dot(p_proj,e)) > 0.5*pi :	
							delta_long = - delta_long
							
						target_long = sc_long + delta_long
									
						if target_long < -180:
							target_long += 360
						if target_long > 180:
							target_long -= 360
						
					
					# calculate image incident angle from target location -> same as satellite elevation from the target's point of view. 
					# Directly overhead = 90 deg
					sigma = ""
					if target_long != "N/A" and target_lat != "N/A" :
						target = Topos(target_lat, target_long)
						line_of_sight = opssat - target
						elevation, azimuth, dist = line_of_sight.at(t_sf).altaz()
						if elevation.degrees >= 0 and elevation.degrees <= 90 :
							sigma = elevation.degrees
					
					
					### Calculate expected photo diode angles from attitude in order to validate attitude against measured data ###
					
					
					# and expected photo diode angles 
					q_pd_xPlus = max(0,90-180.0 /pi * acos(clip(dot(b,ECIx),-1,1)))
					q_pd_xMinus = max(0,90-180.0 /pi * acos(clip(dot(b,-ECIx),-1,1)))
					q_pd_yPlus= max(0,90-180.0 /pi * acos(clip(dot(b,ECIy),-1,1)))
					q_pd_yMinus = max(0,90-180.0 /pi * acos(clip(dot(b,-ECIy),-1,1)))
					q_pd_zPlus= max(0,90-180.0 /pi * acos(clip(dot(b,ECIz),-1,1)))
					q_pd_zMinus = max(0,90-180.0 /pi * acos(clip(dot(b,-ECIz),-1,1)))					
		
			f_att.write (timestamp + "," + str(delta_fwd)+ "," + str(delta_side)  + "," + str(Ydev)  + "," + str(beta)+ "," + str(gamma) + "," + str(rho) + "," + str(sigma)+ "\n" )
			f_loc.write (timestamp + "," + str(sc_lat)+ "," + str(sc_long) + "," + str(target_lat) + "," + str(target_long)+ "\n" )
			f_deb.write (timestamp + "," + str(q0) + ","  + str(q1) + ","  + str(q2)  + "," + str(q3)  + "," + "," + str(q_pd_xPlus)+ "," + str(q_pd_xMinus)+ "," + str(q_pd_yPlus)+ "," + str(q_pd_yMinus)+ "," + str(q_pd_zPlus)+ "," + str(q_pd_zMinus)  + "," + str(pd_xPlus)+ "," + str(pd_xMinus)+ "," + str(pd_yPlus)+ "," + str(pd_yMinus)+ "," + str(pd_zPlus)+ "," + str(pd_zMinus) +"\n" )
		
		
		
		
		if "# DATE TIME" in line:
			header = False
		
		
	f_in.close()
	f_att.close()
	f_loc.close()
	f_deb.close()
	
if __name__ == "__main__":
	parse_TM("input_data.csv")
