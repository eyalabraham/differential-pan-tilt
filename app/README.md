# Differential pan-tilt applications

This repository contains Python 3 library modules and applications for the Pan-Tile system. This collection is an improved version of what I used for my [robotics arm project](https://sites.google.com/site/eyalabraham/robotic-arm).

The applications connect to the pan-tilt system controller through a simple serial console or in a 'remote' mode. This allows the applications to invoke the pan-tilt [controller CLI](..\controller\ptctrl.c).

## Applications

- ```home.py``` simple homing command
- ```test1.py``` a conical scan of the sensor mount
- ```test2.py``` a series of pan-tilt commands
- ```camtest.py``` OpenCV webcam capture and display test
- ```objtracker.py``` object tracker with webcam
- ```facetrack.py``` face or faces detection and tracking with webcam

## Library modules

- ```ptcmd.py``` command interface to the pan-tilt controller
- ```pyik.py``` inverse kinematics model for the pan-tilt system

## Dependencies

- Python 3.6.9
- ```matplotlib``` requires a GUI rendering backend such as ```python3-tk``` that may not be installed by default
- See ```requirements.txt``` file

