#!/usr/bin/env python3

from math import acos, asin, pi, atan, cos, sin, degrees

import ephem
from numpy import dot
from numpy.linalg import norm
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


class SatPos:
    """
    Stores cartesian coordinates.
    """

    def __init__(self, ra, dec, altitude):
        """ Computes the cartesian coordinates from equatorial coordinates.

        Parameters
        ----------
        ra : float
            right inclination
        dec : float
            declination
        """
        radius = (altitude + ephem.earth_radius) / 1000.0
        self.x = radius * cos(dec) * cos(ra)
        self.y = radius * cos(dec) * sin(ra)
        self.z = radius * sin(dec)


class SatState:
    """
    Stores the satellite state at a specific time. All coordinates are in ECI frame.
    """

    def __init__(self, position, lla, quat_attitude, euler_attitude, nadir, angle_to_nadir, timestamp):
        """
        Stores the given state of the satellite at the specified time.

        Parameters
        ----------
        position: SatPos
            Cartesian coordinates of the satellite in ECI frame
        lla: tuple(float)
            Latitude (+N), Longitude (+E) and elevation above sea level (m)
        quat_attitude: tuple(float)
            Satellite attitude in ECI frame: (q0, q1, q2, q3)
        euler_attitude: tuple(float)
            Satellite attitude in degrees in ECI frame: (roll, pitch, yaw)
        nadir: list(float)
            Nadir vector
        angle_to_nadir: float
            Total angle between -Z axis of the satellite and nadir vector in degrees
        timestamp: datetime
            Timestamp of the satellite state
        """
        self.position = position
        self.lla = lla
        self.quat_attitude = quat_attitude
        self.euler_attitude = euler_attitude
        self.nadir = nadir
        self.angle_to_nadir = angle_to_nadir
        self.timestamp = timestamp

    def to_attitude_string(self):
        """ Returns a human readable message containing the attitude data of the satellite.

        Returns
        -------
        The message (String)
        """
        # compute earth pointing flag
        pointing_to_earth = True
        if self.angle_to_nadir > 90:
            pointing_to_earth = False

        return "Satellite state at {}\n" \
               "attitude (roll, pitch, yaw): {}\n" \
               "angle to nadir: {}\n" \
               "pointing to earth: {}".format(
            self.timestamp,
            self.euler_attitude,
            self.angle_to_nadir,
            pointing_to_earth)


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

    q0, q1, q2, q3 = quat
    if not (q0 == 0 and q1 == 0 and q2 == 0 and q3 == 0):
        ### process quaternions ###
        # pyquaternion uses the convention (scalar, [vector]). Same as OBSW.
        q = Quaternion(q0, q1, q2, q3)
        q_ = q.inverse

        # quaternions representing the S/C body frame axes
        sc_body_x = Quaternion(0, 1, 0, 0)  # quaternions representing the S/C body frame axes
        sc_body_y = Quaternion(0, 0, 1, 0)
        sc_body_z = Quaternion(0, 0, 0, 1)
        q_eci = q * sc_body_x * q_  # transformation from body frame to ECI
        eci_x = [q_eci[-3], q_eci[-2], q_eci[-1]]  # X axis
        eci_x = eci_x / norm(eci_x)

        q_eci = q * sc_body_y * q_
        eci_y = [q_eci[-3], q_eci[-2], q_eci[-1]]  # Y axis
        eci_y = eci_y / norm(eci_y)

        q_eci = q * sc_body_z * q_
        eci_z = [q_eci[-3], q_eci[-2], q_eci[-1]]  # Z axis
        eci_z = eci_z / norm(eci_z)

        # now we have the S/C body z axis expressed in ECI coordinates -> eci_z

        # calculate nadir direction
        OPS = ephem.readtle(TLE.line1, TLE.line2, TLE.line3)
        OPS.compute(timestamp)
        lla = (degrees(OPS.sublat), degrees(OPS.sublong), OPS.elevation)
        sat_pos = SatPos(float(OPS.ra), float(OPS.dec), float(OPS.elevation))
        sat_pos_l = [sat_pos.x, sat_pos.y, sat_pos.z]

        nadir = -1 * (sat_pos_l / norm(sat_pos_l))

        # total deviation angle between -Z and nadir
        angle_to_nadir = degrees(acos(dot(-eci_z, nadir)))

        # calculate euler angles roll => x, pitch => y, yaw => z
        # roll
        roll = 2 * (q0 * q1 + q2 * q3)
        roll = roll / (1 - 2 * (q1 ** 2 + q2 ** 2))
        roll = 180.0 / pi * atan(roll)
        if abs(q1) >= 0.71:
            roll = 180 + roll

        # pitch
        pitch = 2 * (q0 * q2 - q3 * q1)
        pitch = 180.0 / pi * asin(pitch)
        if abs(q2) >= 0.71:
            pitch = 180 - pitch

        # yaw
        yaw = 2 * (q0 * q3 + q1 * q2)
        yaw = yaw / (1 - 2 * (q2 ** 2 + q3 ** 2))
        yaw = 180.0 / pi * atan(yaw)
        if abs(q3) >= 0.71:
            yaw = 180 + yaw

        return SatState(sat_pos, lla, quat, (roll, pitch, yaw), nadir, angle_to_nadir, timestamp)


def create_sat_state_generator(timestamped_quats, tle, step=1):
    """ Creates a simple satellite state generator using the prodived TLE and timestamped quaternions.

    Parameters
    ----------
    timestamped_quats : list(tuple(datetime, tuple(float))
        the list of timestamped quaternions
    tle : TLE
        the TLE
    step: int
        a step of 1 means we return every quaternions, 2 means we return 1 quaternions every 2 quaternions and so on.
        Default value is 1.
    """
    for i in range(0, len(timestamped_quats), step):
        yield compute_sat_state(timestamped_quats[i][0], timestamped_quats[i][1], tle)
