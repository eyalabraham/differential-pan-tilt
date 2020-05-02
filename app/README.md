# Differential pan-tilt applications

This repository contains Python 3 library modules and applications for the Pan-Tile system. This collection is an improved version of what I used for my [robotics arm project](https://sites.google.com/site/eyalabraham/robotic-arm).

The applications connect to the pan-tilt system controller through a simple serial console or in a 'remote' mode. This allows the applications to invoke the pan-tilt [controller CLI](..\controller\ptctrl.c).

## Applications

## Library modules

- ```ptcmd.py``` command interface to the pan-tilt controller
- ```pyik.py``` inverse kinematics model for the pan-tilt system

## Dependencies

See ```requirements.txt``` file

