# Moonraker Rover software stack

Moonraker project with ROS2 Humble

## Git maintenance rule
- Always put the stable code in `main` branch
- If you want to add any features, create `feature` branch and merge it to `main` later
- If you fixed the bug and want to update, make `devel/bug` branch and merge it with `main` branch
- Send pull request before merge to main branch (I added branch rule, so you cannot directly merge with main branch.)
- Only upload source code and configuration files (no heavy data!)

## Environment
The repository mainly depend on
- Robot computer: NUC13 (Ubuntu 22.04, ROS2 Humble)
- Host PC: ThinkPad (Ubuntu 20.04, ROS2 Foxy)
- MCU: ESP32

## System diagram (WIP)