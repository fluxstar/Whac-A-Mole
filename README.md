# Whac-A-Mole Bot Simulation

Run the following after pulling (while in `/whac-a-mole`):
- `catkin_make`
- `source devel/setup.bash`
- `roslaunch whac-a-mole_description display.launch`

While that is running, open a new terminal in the same directory and run the following: 
- `source devel/setup.bash`
- `rosrun whac-a-mole_description whac-a-mole.py`

Use the `1-4` keys to push down the corresponding button (left to right)
Press the `` ` `` key for a secret!

All unique code can be found in the `/whac-a-mole/whac-a-mole_description` directory.
See `scripts` for Python code, `urdf` for model/joint setup