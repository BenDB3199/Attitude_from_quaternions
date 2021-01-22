import configparser
from datetime import datetime

from attitude_processor.satellite_state import TLE


def load_tle_and_quat(config_file):
    """ Loads TLE and quaternion using paths provided in the given configuration file.

    Parameters
    ----------
    config_file : string
        path to the configuration file to use

    Returns
    -------
    timestamp (datetime), quaternion (list(float)), TLE
    """
    config = configparser.ConfigParser()
    config.read(config_file)

    # load quaternion file
    f_q_path = config['conf']['quat_path']
    f_q = open(f_q_path, 'r')
    f_q_line = f_q.read()
    f_q_line = f_q_line.split()

    # parse quaternion file
    date_time_str = f_q_line[0] + ' ' + f_q_line[1]
    date_time_obj = datetime.strptime(date_time_str, '%Y-%m-%d %H:%M:%S.%f')
    quat = [float(f_q_line[2]),
            float(f_q_line[3]),
            float(f_q_line[4]),
            float(f_q_line[5])]

    # load TLE file
    tle_path = config['conf']['tle_path']
    f_tle = open(tle_path, 'r')
    f_tle_lines = f_tle.readlines()

    # parse TLE file
    tle = TLE(f_tle_lines[0], f_tle_lines[1], f_tle_lines[2])

    return date_time_obj, quat, tle
