###########################################################
#
# test1.py
#
#   Pan-tilt controller test.
#   Runs a conical scan of the sensor mount.
#
#   April 18, 2020
#
###########################################################

import ptcmd
import ptik as system
import time
import math
import sys

a = 0
delta_a = 2 * math.pi / 50

with ptcmd.PTCMD(baud=57600) as controller:

    #controller.home()
    for loop in range(0, 1):
        for step in range(0, 50):
            t = math.sin(a) * 15 - 45
            p = math.cos(a) * 30
            a += delta_a

            current_position = controller.get_position()
            pan_steps, tilt_steps, pan_rate, tilt_rate = system.move_to_angle(p, t, current_position[0], current_position[1])

            #print(pan_rate, pan_steps, tilt_rate, tilt_steps)
            controller.sync(pan_rate, pan_steps, tilt_rate, tilt_steps)
            controller.wait('all')

