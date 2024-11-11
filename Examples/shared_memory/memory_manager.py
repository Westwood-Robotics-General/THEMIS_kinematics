#!usr/bin/env python
__author__    = "Westwood Robotics Corporation"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2024 Westwood Robotics Corporation"
__date__      = "October 23, 2024"
__project__   = "BRUCE"
__version__   = "0.0.7"
__status__    = "Product"

'''
Example of creating shared memory segments
'''

import numpy as np
from Library.SHARED_MEMORY import Manager as shmx

# Time state
TIME_STATE = shmx.SHMEMSEG(robot_name='EXAMPLE', seg_name='TIME_STATE', init=False)
TIME_STATE.add_block(name='time', data=np.array([1.234]))

# Joint state
JOINT_STATE = shmx.SHMEMSEG(robot_name='EXAMPLE', seg_name='JOINT_STATE', init=False)
JOINT_STATE.add_block(name='joint_positions',  data=np.array([-0.1, +0.2]))
JOINT_STATE.add_block(name='joint_velocities', data=np.array([+0.3, -0.4]))


def init():
    """Init if main"""
    TIME_STATE.initialize  = True
    JOINT_STATE.initialize = True


def connect():
    """Connect and create segment"""
    TIME_STATE.connect_segment()
    JOINT_STATE.connect_segment()


if __name__ == '__main__':
    init()
    connect()
else:
    connect()