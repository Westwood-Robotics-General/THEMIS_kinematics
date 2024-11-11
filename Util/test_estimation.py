#!usr/bin/env python
__author__    = "Westwood Robotics Corporation"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2024 Westwood Robotics Corporation"
__date__      = "October 23, 2024"
__project__   = "BRUCE"
__version__   = "0.0.7"
__status__    = "Product"

'''
Script for monitoring estimator data (need to have run_estimation.py concurrently running)
'''

import time
import Settings.BRUCE_data as RDS
from Settings.BRUCE_macros import *


if __name__ == '__main__':
    Bruce = RDS.BRUCE()

    while True:
        Bruce.update_robot_status()

        print("       Base Position: {:+.2f} {:+.2f} {:+.2f}".format(Bruce.p_wb[0],    Bruce.p_wb[1],    Bruce.p_wb[2]))
        print("       Base Velocity: {:+.2f} {:+.2f} {:+.2f}".format(Bruce.v_wb[0],    Bruce.v_wb[1],    Bruce.v_wb[2]))
        print("        CoM Position: {:+.2f} {:+.2f} {:+.2f}".format(Bruce.p_wg[0],    Bruce.p_wg[1],    Bruce.p_wg[2]))
        print("        CoM Velocity: {:+.2f} {:+.2f} {:+.2f}".format(Bruce.v_wg[0],    Bruce.v_wg[1],    Bruce.v_wg[2]))
        print("    Base Orientation: {:+.2f} {:+.2f} {:+.2f}".format(Bruce.R_wb[0, 0], Bruce.R_wb[0, 1], Bruce.R_wb[0, 2]))
        print("                      {:+.2f} {:+.2f} {:+.2f}".format(Bruce.R_wb[1, 0], Bruce.R_wb[1, 1], Bruce.R_wb[1, 2]))
        print("                      {:+.2f} {:+.2f} {:+.2f}".format(Bruce.R_wb[2, 0], Bruce.R_wb[2, 1], Bruce.R_wb[2, 2]))
        print("")
        print("Right Ankle Position: {:+.2f} {:+.2f} {:+.2f}".format(Bruce.p_wa_r[0],  Bruce.p_wa_r[1],  Bruce.p_wa_r[2]))
        print(" Left Ankle Position: {:+.2f} {:+.2f} {:+.2f}".format(Bruce.p_wa_l[0],  Bruce.p_wa_l[1],  Bruce.p_wa_l[2]))
        for _ in range(10):
            print('\033[1A', end='\x1b[2K')

        time.sleep(0.01)