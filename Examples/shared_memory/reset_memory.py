#!usr/bin/env python
__author__    = "Westwood Robotics Corporation"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2024 Westwood Robotics Corporation"
__date__      = "October 23, 2024"
__project__   = "BRUCE"
__version__   = "0.0.7"
__status__    = "Product"

'''
Reset shared memory to default
'''

import Examples.shared_memory.memory_manager as mm


if __name__ == "__main__":
    mm.TIME_STATE.set(opt='default')
    mm.JOINT_STATE.set(opt='default')