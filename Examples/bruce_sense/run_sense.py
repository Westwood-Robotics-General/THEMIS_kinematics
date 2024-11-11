#!usr/bin/env python
__author__    = "Westwood Robotics Corporation"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2024 Westwood Robotics Corporation"
__date__      = "October 23, 2024"
__project__   = "BRUCE"
__version__   = "0.0.7"
__status__    = "Product"

'''
Get IMU/foot contact data and set cooling speed
'''

import time
from Settings.BRUCE_macros import *
from Library.BRUCE_SENSE import Manager as sense_manager


if __name__ == '__main__':
    # PICO setup
    sm = sense_manager.SenseManager(port=PICO_port, baudrate=PICO_baudrate)

    # Set to nominal mode to read data
    sm.send_data(pico_mode='nominal')
    time.sleep(1)

    # Read data
    while not sm.read_data():  # wait for new data
        pass
    print("Acceleration: {:+.2f} {:+.2f} {:+.2f}".format(sm.accel[0], sm.accel[1], sm.accel[2]))
    print("Angular Rate: {:+.2f} {:+.2f} {:+.2f}".format(sm.omega[0], sm.omega[1], sm.omega[2]))
    print("Foot Contact: {:.0f} {:.0f} {:.0f} {:.0f}".format(sm.foot_contact[0], sm.foot_contact[1], sm.foot_contact[2], sm.foot_contact[3]))
    time.sleep(1)

    # Set cooling speed (only work in nominal mode)
    sm.send_data(pico_mode='nominal', cooling_speed=2)  # range: 0-5
    time.sleep(1)

    # Set to idle mode
    sm.send_data(pico_mode='idle')