###########################################################
#
# roomscan.py
#
#   Scan and map a room with an ultrasonic sensor mounted
#   on the pan-tilt system.
#
#   May 9, 2020
#
###########################################################

import math
import time

import matplotlib.pyplot as plt
import numpy as np

import ptcmd
import ptik as system

###############################################################################
#
# main()
#
def main():
    """
    Scan and map a room with an ultrasonic sensor mounted
    on the pan-tilt system.
    """

    r = np.arange(-90, 90, 2)
    tilt = 22.5

    with ptcmd.PTCMD(baud=57600) as controller:

        controller.home()

        for pan_angle in r:

            current_position = controller.get_position()
            pan_steps, tilt_steps, pan_rate, tilt_rate = system.move_to_angle(pan_angle, tilt, current_position[0], current_position[1], base_rate = 1000)
            controller.sync(pan_rate, pan_steps, tilt_rate, tilt_steps)
            controller.wait('all')

            time.sleep(2)

            distances = []
            for i in range(0,12):
                time.sleep(1)
                distances.append(controller.get_analog_sensor(1))

            distances.sort()
            distance = sum(distances[1:-1]) / len(distances[1:-1])

            x = math.sin(pan_angle*math.pi/180) * distance
            y = math.cos(pan_angle*math.pi/180) * distance

            print(distances, distance, pan_angle, x, y)

            plt.plot(x, y, 'b+')

        plt.show()



#
# Startup
#
if __name__ == '__main__':
    main()
