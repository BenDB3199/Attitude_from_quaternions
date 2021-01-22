from ground_visualization import visualizer
from attitude_processor.satellite_state import compute_sat_state
from data.loader import load_tle_and_quat

# keep this import
from ground_visualization import plotters_utils


def run():
    """
    Runs the application.

    TODO arg parser to select what is wanted (print, plots, both ...)
    """
    date_time, quat, tle = load_tle_and_quat('config.ini')
    sat_state = compute_sat_state(date_time, quat, tle)

    print(sat_state.to_attitude_string())

    v = visualizer.AttitudeVisualizer()
    v.update(sat_state)
    v.show()


if __name__ == '__main__':
    run()
