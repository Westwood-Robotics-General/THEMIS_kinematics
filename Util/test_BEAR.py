#!usr/bin/env python
__author__    = "Westwood Robotics Corporation"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2024 Westwood Robotics Corporation"
__date__      = "October 23, 2024"
__project__   = "BRUCE"
__version__   = "0.0.7"
__status__    = "Product"

'''
Test communication with BEAR actuators
'''

import time
from pybear import Manager
from Settings.BRUCE_macros import *

if __name__ == '__main__':
    bear = Manager.BEAR(port=BEAR_port, baudrate=BEAR_baudrate)

    bear_list = [int(idx) for idx in input("Enter the list of BEAR IDs (e.g., 1 2 6): ").split()]
    while True:
        loop_start_time = time.time()
        data = bear.bulk_read(bear_list, ['present_position', 'present_velocity', 'present_iq'])
        print(1. / (time.time() - loop_start_time))