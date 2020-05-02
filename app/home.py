###########################################################
#
# home.py
#
#   Pan-tilt system home.
#
#   May 2, 2020
#
###########################################################

import ptcmd

with ptcmd.PTCMD(baud=57600) as controller:
    controller.home()


