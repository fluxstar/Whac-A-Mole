# Whac-A-Mole

Run the following after pulling (while in `/whac-a-mole`):
`catkin_make`
`source devel/setup.bash`
`roslaunch whac-a-mole_description display.launch`

(in a new terminal): 
`source devel/setup.bash`
`rosrun whac-a-mole_description whac-a-mole.py`

Use `1-4` keys to push down the corresponding button (left to right)
Press the `~` key for a secret!

All unique code can be found in the `/whac-a-mole/whac-a-mole_description` directory
see `scripts` for Python code, `urdf` for model/joint setup