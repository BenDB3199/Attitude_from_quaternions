import configparser
import os
import socket
import threading

from attitude_processor.satellite_state import compute_sat_state, create_sat_state_generator
from data.loader import load_tle_and_quat, load_tle, parse_quat_line
from ground_visualization import visualizer

# Path of the folder containing this script
_DIR_NAME = os.path.dirname(__file__)

# Name of the configuration file
_CONFIG_FILE = 'configuration/config_ground.ini'
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


def _start_single_quat_mode(tle_file_path, quat_file_path):
    """ Starts the visualizer in single quaternion mode: displays a single satellite attitude
    computed using a TLE file and the first line of the quaternion file.

     Parameters
    ----------
    tle_file_path : string
        path to the TLE file to load
    quat_file_path : string
        path to the timestamped quaternion file to load
    """
    timestamped_quats, tle = load_tle_and_quat(tle_file_path, quat_file_path)
    sat_state = compute_sat_state(timestamped_quats[0][0], timestamped_quats[0][1], tle)

    v = visualizer.AttitudeVisualizer()
    v.update(sat_state)
    v.show()


def _start_multiple_quat_mode(tle_file_path, quat_file_path, start_line, end_line, step, interval, video_export):
    """ Starts the visualizer in multiple quaternions mode: successively displays (frame by frame) satellite
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
        time in milliseconds between frames
    video_export: bool
        if true, exports an MP4 video instead of showing the visualizer
    """
    timestamped_quats, tle = load_tle_and_quat(tle_file_path, quat_file_path, start_line=start_line, end_line=end_line)
    sat_state_generator = create_sat_state_generator(timestamped_quats, tle, step=step)

    v = visualizer.AttitudeVisualizer()
    v.animate(sat_state_generator, interval=interval, save=video_export)
    v.show()


def _background_udp_server(hostname, port, visualizer, tle):
    """ Starts a UDP server in daemon thread. The server updates the visualizer with the satellite state computed
    from the given TLE and the received quaternions.

    Parameters
    ----------
    hostname: string
        hostname to use for the UDP server
    port: int
        port to use for the UDP server
    visualizer: AttitudeVisualizer
        the visualizer
    tle: TLE
        the TLE to use
    """
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind((hostname, port))

    print("Listening on UDP {}:{}".format(hostname, port))

    while True:
        data, addr = s.recvfrom(100)
        print("==========================")
        print("From {} received {} bytes:\n{}".format(addr, len(data), data))

        decoded_data = data.decode('utf-8')
        print("-----------")
        print("Decoded bytes using UTF-8 encoding:\n{}".format(decoded_data))

        timestamp, quat = parse_quat_line(decoded_data)
        print("-----------")
        print("Extracted timestamp and quaternion:\n{}\n{}".format(timestamp, quat))

        sat_state = compute_sat_state(timestamp, quat, tle)
        visualizer.update(sat_state)


def _start_server_mode(tle_file_path, hostname, port):
    """ Starts the visualizer in server mode: runs forever and displays (frame by frame) satellite
    attitudes computed using a TLE file and the quaternions received over UDP.

     Parameters
    ----------
    tle_file_path : string
        path to the TLE file to load
    hostname: string
        hostname to use for the UDP server
    port: int
        port to use for the UDP server
    """
    tle = load_tle(tle_file_path)
    v = visualizer.AttitudeVisualizer()

    t = threading.Thread(target=_background_udp_server, args=[hostname, port, v, tle])
    t.setDaemon(True)
    t.start()

    v.show()


def run(config_file_path):
    """ Runs the ground visualizer using the provided configuration file.

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
    if mode == 'SINGLE_QUAT':
        quat_file_path = config[mode]['quat_path']
        _start_single_quat_mode(tle_file_path, quat_file_path)
    elif mode == 'MULTIPLE_QUAT':
        quat_file_path = config[mode]['quat_path']
        step = int(config[mode]['step'])
        start_line = int(config[mode]['start_line'])
        end_line = int(config[mode]['end_line'])
        interval = int(config[mode]['interval'])
        video_export = config[mode]['video_export'] == 'True'
        _start_multiple_quat_mode(tle_file_path, quat_file_path, start_line, end_line, step, interval, video_export)
    elif mode == 'SERVER':
        hostname = config[mode]['hostname']
        port = int(config[mode]['port'])
        _start_server_mode(tle_file_path, hostname, port)
    else:
        print("Error: unknown mode {}".format(mode))


if __name__ == '__main__':
    run(_CONFIG_FILE_PATH)
