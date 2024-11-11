#!usr/bin/env python
__author__    = "Westwood Robotics Corporation"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2024 Westwood Robotics Corporation"
__date__      = "October 23, 2024"
__project__   = "BRUCE"
__version__   = "0.0.7"
__status__    = "Product"

'''
Launch Configuration
'''

SIMULATION = False   # if in simulation or not
GAMEPAD    = False   # if using gamepad or not
ESTIMATION = True    # if running estimation or not (for simulation)
DEMO       = True    # if running demo or op

HARDWARE = not SIMULATION
if HARDWARE:
    ESTIMATION = True

if __name__ == '__main__':
    print(DEMO)
