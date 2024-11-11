#!usr/bin/env python
__author__    = "Westwood Robotics Corporation"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2024 Westwood Robotics Corporation"
__date__      = "October 23, 2024"
__project__   = "BRUCE"
__version__   = "0.0.7"
__status__    = "Product"

'''
Script for monitoring sense data (need to have run_sense.py concurrently running)
'''

import time
import Settings.BRUCE_data as RDS
from Settings.BRUCE_macros import *


if __name__ == '__main__':
    Bruce = RDS.BRUCE()

    while True:
        Bruce.update_sense_status()

        print("Right Foot Contact: {:.0f} {:.0f}".format(Bruce.foot_contacts[0], Bruce.foot_contacts[1]))
        print(" Left Foot Contact: {:.0f} {:.0f}".format(Bruce.foot_contacts[2], Bruce.foot_contacts[3]))
        print("")
        print("      Acceleration: {:+.2f} {:+.2f} {:+.2f}".format(Bruce.imu_accel[0], Bruce.imu_accel[1], Bruce.imu_accel[2]))
        print("      Angular Rate: {:+.2f} {:+.2f} {:+.2f}".format(Bruce.imu_omega[0], Bruce.imu_omega[1], Bruce.imu_omega[2]))
        print("")
        for _ in range(6):
            print('\033[1A', end='\x1b[2K')

        time.sleep(0.01)