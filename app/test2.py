###########################################################
#
# test2.py
#
#   Pan-tilt controller test.
#   Runs a series of pan-tilt commands.
#
#   April 18, 2020
#
###########################################################

import ptcmd
import ptik as system
import time
import math
import sys

def go_to(pan, tilt):
    """Go to a pan-tilt angle."""

    global system, controller

    print(f'target position  ({pan}, {tilt}) [degrees]')

    # Current position
    current_position = controller.get_position()
    print(f'current position {current_position} [steps]')

    # Target position
    pan_steps, tilt_steps, pan_rate, tilt_rate = system.move_to_angle(pan, tilt, current_position[0], current_position[1], base_rate = 1000)

    # Delta more and rates
    print(f'sync() solution pan:({pan_rate}, {pan_steps}) tilt:({tilt_rate}, {tilt_steps})')

    # Move
    controller.sync(pan_rate, pan_steps, tilt_rate, tilt_steps)
    controller.wait('all')


#
# Startup
#
if __name__ == '__main__':
    controller = ptcmd.PTCMD(baud=57600)
    controller.home()

    for loop in range(0, 1):
        go_to(-45, +45)
        go_to(+45, +45)
        go_to(-45, -45)
        go_to(+45, -45)
        go_to(0, 0)

