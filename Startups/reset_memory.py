#!usr/bin/env python
__author__    = "Westwood Robotics Corporation"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2024 Westwood Robotics Corporation"
__date__      = "October 23, 2024"
__project__   = "BRUCE"
__version__   = "0.0.7"
__status__    = "Product"

'''
Reset shared memory
'''

import Startups.memory_manager as MM


if __name__ == "__main__":
    MM.THREAD_STATE.set(opt='default')
    MM.SIMULATOR_STATE.set(opt='default')
    MM.SENSE_STATE.set(opt='default')
    MM.GAMEPAD_STATE.set(opt='default')
    MM.LEG_STATE.set(opt='default')
    MM.LEG_COMMAND.set(opt='default')
    MM.ARM_STATE.set(opt='default')
    MM.ARM_COMMAND.set(opt='default')
    MM.ESTIMATOR_STATE.set(opt='default')
    MM.ESTIMATOR_COMMAND.set(opt='default')
    MM.PLANNER_COMMAND.set(opt='default')
    MM.USER_COMMAND.set(opt='default')