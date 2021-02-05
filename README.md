# Attitude_from_quaternions
The code takes in quaternions  and a timestamp along with TLE data from the OPS-SAT and outputs satellite attitude information like roll, pitch, yaw and angle to nadir.

### Installation
1. Go to the application root folder: `cd attitude_app/`
2. Set up the virtual environment (venv)
    1. Install virtualenv: `pip3 install --user virtualenv`
    2. Create the venv: `python3 -m venv venv`
    3. Start the venv: `source venv/bin/activate`
    4. Install dependencies in the venv: `pip3 install -r requirements.txt`
3. Install cartopy package (only if you want to run the ground visualizer)
```
# install non-python dependencies, example is for Ubuntu
# see https://scitools.org.uk/cartopy/docs/latest/installing.html for details
sudo apt install proj-bin libgeos-dev

# install python dependency
pip3 install --no-binary shapely shapely

# install cartopy python package
pip3 install cartopy
```

### Configuration
The configuration files are located in the `attitude_app/configuration/`
* `config.ini` configures the space app `app.py`
* `config_ground.ini` configures the ground visualizer `app_ground.py`

### Running
1. If not done already, follow the [Installation](#Installation) instructions
2. [Configure](#Configuration) the app as you like
3. Run the desired app:
    * `python3 app.py` to run the space app
    * `python3 app_ground.py` to run the ground visualizer


