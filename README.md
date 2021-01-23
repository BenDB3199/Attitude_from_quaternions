# Attitude_from_quaternions
The code takes in quaternions  and a timestamp along with TLE data from the OPS-SAT and outputs satellite attitude information like roll, pitch, yaw and angle to nadir.

### Installation
1. Go to the application root folder: `cd attitude_app/`
2. Install virtualenv: `pip3 install --user virtualenv`
3. Create the virtual environment: `python3 -m venv venv`
4. Start the virtual environment: `source venv/bin/activate`
5. Install package dependencies: `pip3 install -r requirements.txt`

### Configuration
The configuration file `config.ini` is available to select the data used by the application (quaternion and TLE).
Default file used are `attitude_app/data/quaternion.txt` and `attitude_app/data/tle.txt`.

### Running
1. If not there already, go to the application root folder: `cd attitude_app/`
2. Run the app: `python3 app.py`


