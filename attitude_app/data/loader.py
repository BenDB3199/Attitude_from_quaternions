import configparser
from datetime import datetime

from attitude_processor.satellite_state import TLE


def load_tle_and_quat(config_file, nb_lines=1):
    """ Loads TLE and the requested number of timestamped quaternions using paths provided in the given configuration file.

    Parameters
    ----------
    config_file : string
        path to the configuration file to use
    nb_lines : int
        the number of lines to read in the quaternion file. Default value is 1. Set it to -1 to read all lines.
    Returns
    -------
    list(tuple(datetime, tuple(float)), TLE
    """
    config = configparser.ConfigParser()
    config.read(config_file)

    # load quaternion file
    timestamped_quats = load_quaternions(config['conf']['quat_path'], nb_lines=nb_lines)

    # load TLE file
    tle = load_tle(config['conf']['tle_path'])

    return timestamped_quats, tle

def load_quaternions(quat_file_path, nb_lines=1):
    """ Loads the requested number of timestamped quaternions from a text file.

    Parameters
    ----------
    quat_file_path : string
        path to the timestamped quaternions file to load
    nb_lines : int
        the number of lines to read in the file. Default value is 1. Set it to -1 to read all lines.

    Returns
    -------
    list(tuple(datetime, tuple(float)) the list of timestamped quaternions
    """
    timestamped_quats = []

    with open(quat_file_path, 'r') as quat_file:
        quat_line = quat_file.readline()
        lines_read = 0

        # read lines till end of file or line count reached
        while quat_line and (lines_read < nb_lines or nb_lines==-1):
            timestamp, quat = parse_quat_line(quat_line)
            timestamped_quats.append((timestamp, quat))
            lines_read += 1
            quat_line = quat_file.readline()

    return timestamped_quats

def parse_quat_line(quat_line):
    """ Parses a timestamped quaternion line.

    Parameters
    ----------
    quat_line : string
        the quaternion line with the following expected format: '%Y-%m-%d %H:%M:%S' q0 q1 q2 q3
    Returns
    -------
    datetime, tuple(float) the timestamped quaternion
    """
    quat_line = quat_line.split(' ')
    date_time_str = quat_line[0] + ' ' + quat_line[1]

    timestamp = datetime.strptime(date_time_str, '%Y-%m-%d %H:%M:%S')
    quat = (float(quat_line[2]),
            float(quat_line[3]),
            float(quat_line[4]),
            float(quat_line[5]))

    return timestamp, quat


def load_tle(tle_file_path):
    """ Loads a TLE file.

    Parameters
    ----------
    tle_file_path : string
        path to the TLE file to load
    Returns
    -------
    TLE
    """
    with open(tle_file_path, 'r') as tle_file:
        tle_file_lines = tle_file.readlines()
        return TLE(tle_file_lines[0], tle_file_lines[1], tle_file_lines[2])
