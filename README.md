# Moonraker Rover software stack

All softwares are stored here. 
For moonraker, we use ROS2 foxy. 

## Git maintenance rule
- Always put the stable code in `main` branch
- If you want to add any features, create `feature` branch and merge it to `main` later
- If you fixed the bug and want to update, make `devel/bug` branch and merge it with `main` branch
- Try to send pull request before merge
- Only upload source code and configuration files (no heavy data!)

## Dependencies
The repository mainly depend on
- ROS2 foxy (C++ or python)
- micro-ros-arduino (Arduino language)