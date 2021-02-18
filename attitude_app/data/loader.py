from datetime import datetime

from attitude_processor.satellite_state import TLE


def load_tle_and_quat(tle_file_path, quat_file_path, start_line=1, end_line=1):
    """ Loads TLE and the timestamped quaternions from the requested lines of the quaternions file.

    Parameters
    ----------
    tle_file_path : string
        path to the TLE file to load
    quat_file_path : string
        path to the timestamped quaternions file to load
    start_line: int
        first line to read in timestamped quaternions file
    end_line: int
        last line to read in timestamped quaternions file, set it to -1 to go to end of file

    Returns
    -------
    list(tuple(datetime, tuple(float)), TLE
    """
    # load quaternion file
    timestamped_quats = load_quaternions(quat_file_path, start_line=start_line, end_line=end_line)

    # load TLE file
    tle = load_tle(tle_file_path)

    return timestamped_quats, tle


def load_quaternions(quat_file_path, start_line=1, end_line=1):
    """ Loads the timestamped quaternions from the requested lines of the provided file.

    Parameters
    ----------
    quat_file_path : string
        path to the timestamped quaternions file to load
    start_line: int
        first line to read in the file
    end_line: int
        last line to read in the file, set it to -1 to go to end of file

    Returns
    -------
    list(tuple(datetime, tuple(float)) the list of timestamped quaternions
    """
    timestamped_quats = []

    with open(quat_file_path, 'r') as quat_file:
        # skip first lines
        for _ in range(start_line - 1):
            next(quat_file)

        quat_line = quat_file.readline()
        line_number = start_line

        # read lines till end of file or end_line
        while quat_line and (line_number <= end_line or end_line == -1):
            timestamp, quat = parse_quat_line(quat_line)
            timestamped_quats.append((timestamp, quat))
            line_number += 1
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
