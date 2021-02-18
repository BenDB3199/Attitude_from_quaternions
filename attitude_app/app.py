import configparser
import os
import time

from attitude_processor.satellite_state import create_sat_state_generator
from data.loader import load_tle_and_quat

# Path of the folder containing this script
_DIR_NAME = os.path.dirname(__file__)

# Name of the configuration file
_CONFIG_FILE = 'configuration/config.ini'
# Absolute path of the configuration file
_CONFIG_FILE_PATH = os.path.join(_DIR_NAME, _CONFIG_FILE)


def _parse_config(config_file_path):
    """ Parses the provided configuration file.

    Parameters
    ----------
    config_file_path: string
        path to the configuration file to use

    Returns
    -------
    the ConfigParser object
    """
    config = configparser.ConfigParser()
    config.read(config_file_path)

    return config


def _start_playback_mode(tle_file_path, quat_file_path, start_line, end_line, step, interval):
    """ Starts space attitude app in multiple quaternions mode: successively prints the satellite
    attitudes computed using a TLE file and the requested lines of the quaternions file.

     Parameters
    ----------
    tle_file_path : string
        path to the TLE file to load
    quat_file_path : string
        path to the timestamped quaternions file to load
    start_line: int
        first line to read in timestamped quaternions file
    end_line: int
        last line to read in timestamped quaternions file
    step: int
        read every step line(s) from start_line to end_line
    interval: int
        time in milliseconds between prints
    """
    timestamped_quats, tle = load_tle_and_quat(tle_file_path, quat_file_path, start_line=start_line, end_line=end_line)
    sat_state_generator = create_sat_state_generator(timestamped_quats, tle, step=step)

    print("=============")
    print("PLAYBACK MODE")
    print("- TLE file: {}".format(tle_file_path))
    print("- quaternions file: {}".format(quat_file_path))
    print("\t - line step:\t{}".format(step))
    print("\t - start line:\t{}".format(start_line))
    print("\t - end line:\t{}".format(end_line))
    print("=============")

    for sat_state in sat_state_generator:
        if sat_state is not None:
            print("----")

            if sat_state != -1:
                print(sat_state.to_attitude_string())
            else:
                print("Invalid SatState")

            time.sleep(interval / 1000.0)


def run(config_file_path):
    """ Runs the space attitude app using the provided configuration file.

    Parameters
    ----------
    config_file_path: string
        path to the configuration file to use
    """
    config = _parse_config(config_file_path)

    # read global configuration
    mode = config['global']['mode']
    tle_file_path = config['global']['tle_path']

    # start depending on mode
    if mode == 'PLAYBACK':
        quat_file_path = config[mode]['quat_path']
        step = int(config[mode]['step'])
        start_line = int(config[mode]['start_line'])
        end_line = int(config[mode]['end_line'])
        interval = int(config[mode]['interval'])
        _start_playback_mode(tle_file_path, quat_file_path, start_line, end_line, step, interval)
    else:
        print("Error: unknown mode {}".format(mode))


if __name__ == '__main__':
    run(_CONFIG_FILE_PATH)
