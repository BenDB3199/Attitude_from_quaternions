#!/usr/bin/env python3

from math import acos, cos, sin, degrees

import ephem
import numpy as np
from numpy import dot
from numpy.linalg import norm
from scipy.spatial.transform import Rotation as ro


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


class SatPos:
    """
    Stores cartesian coordinates.
    """

    def __init__(self, ra, dec):
        """ Computes the cartesian coordinates from equatorial coordinates assuming radius of 1 (normed).

        Parameters
        ----------
        ra : float
            right inclination
        dec : float
            declination
        """
        self.x = cos(dec) * cos(ra)
        self.y = cos(dec) * sin(ra)
        self.z = sin(dec)


class SatState:
    """
    Stores the satellite state at a specific time. All coordinates are in ECI frame.
    """

    def __init__(self, position, attitude, nadir, angle_to_nadir, timestamp):
        """
        Stores the given state of the satellite at the specified time.
        """
        self.position = position
        self.attitude = attitude
        self.nadir = nadir
        self.angle_to_nadir = angle_to_nadir
        self.timestamp = timestamp

    def to_attitude_string(self):
        # convert quaternion to euler angles roll => x, pitch => y, yaw => z
        euler_attitude = self.attitude.as_euler("ZYX", degrees=True)
        x, y, z = self.position.x, self.position.y, self.position.z

        return "Satellite state at {}\n" \
               "position (x, y, z): [{} {} {}]\n" \
               "attitude (roll, pitch, yaw): {}\n" \
               "angle to nadir: {}".format(
            self.timestamp,
            x, y, z,
            euler_attitude,
            self.angle_to_nadir)


def compute_sat_state(timestamp, quat, TLE):
    """ Calculates the satellite state at a given time using the provided quaternion and TLE.

    Parameters
    ----------
    timestamp : datetime
    quat : list(float)
    TLE : TLE

    Returns
    -------
    The satellite state (SatState)
    """

    if not (quat[0] == 0 and quat[1] == 0 and quat[2] == 0 and quat[3] == 0):
        # quaternion parsing with Rotation from SciPy, convention is ([vector], scalar)
        q = ro.from_quat([quat[1], quat[2], quat[3], quat[0]])

        # S/C body frame axes
        sc_body = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
        sc_body_eci = q.apply(sc_body)

        # calculate nadir direction
        OPS = ephem.readtle(TLE.line1, TLE.line2, TLE.line3)
        OPS.compute(timestamp)
        sat_pos = SatPos(float(OPS.ra), float(OPS.dec))
        sat_pos_l = [sat_pos.x, sat_pos.y, sat_pos.z]

        nadir = -1 * (sat_pos_l / norm(sat_pos_l))

        # total deviation angle between -Z and nadir
        angle_to_nadir = degrees(acos(dot(-sc_body_eci[2], nadir)))

        return SatState(sat_pos, q, nadir, angle_to_nadir, timestamp)
