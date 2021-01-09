# Attitude_from_quaternions
The code is supposed to take in quaternions and a timestamp from the OPS-SAT and output attitude information like euler angles.

### Installation of the virtual enviornment
1. Install the virtual environment:     pip3 install --user virtualenv
2. Create the virtal environment:       python3 -m venv venv
3. Start the virtual environment:       source env/bin/activate
4. Install package dependencies:        
pip3 install ephem
pip3 install datetime
pip3 install pyquaternion
pip3 install numpy
5. Create requirements.txt file:        pip freeze > requirements.txt
