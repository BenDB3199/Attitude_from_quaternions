from attitude_processor.satellite_state import compute_sat_state, create_sat_state_generator
from data.loader import load_tle_and_quat
from ground_visualization import visualizer


def run():
    """
    Runs the visualizer once by loading only one timestamped quaternion and a TLE.
    """
    timestamped_quats, tle = load_tle_and_quat('config.ini', nb_lines=1)
    sat_state = compute_sat_state(timestamped_quats[0][0], timestamped_quats[0][1], tle)

    print(sat_state.to_attitude_string())

    v = visualizer.AttitudeVisualizer()
    v.show(sat_state)


def run_on_file():
    """
    Runs the visualizer in animation mode. In this mode, the visualizer updates itself with new satellite state provided
    by a satellite state generator.
    """
    timestamped_quats, tle = load_tle_and_quat('config.ini', nb_lines=-1)
    sat_state_generator = create_sat_state_generator(timestamped_quats, tle, step=1)

    v = visualizer.AttitudeVisualizer()
    v.animate(sat_state_generator, interval=200, save=True)


if __name__ == '__main__':
    """
    TODO arg parser to select what is wanted (print, plots...) and import only required packages.
    """
    # run()
    run_on_file()
