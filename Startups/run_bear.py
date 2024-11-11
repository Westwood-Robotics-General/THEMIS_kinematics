#!usr/bin/env python
__author__    = "Westwood Robotics Corporation"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2024 Westwood Robotics Corporation"
__date__      = "October 23, 2024"
__project__   = "BRUCE"
__version__   = "0.0.7"
__status__    = "Product"

'''
Script for communication with BEAR actuators on BRUCE
'''

import time
import Settings.BRUCE_data as RDS
import Startups.memory_manager as MM
import Library.ROBOT_MODEL.BRUCE_kinematics as kin
from Play.config import *
from termcolor import colored
from Settings.BRUCE_macros import *
from Library.ACTUATORS.BEAR_controller import BEARController


class BEAR_ESTOP_Exception(Exception):
    """
    Raised when E-STOP signal is detected
    """
    pass


class BEAR_ERROR_Exception(Exception):
    """
    Raised when BEAR in error
    """
    pass


class bear_controller:
    def __init__(self, port='/dev/ttyUSB0', baudrate=8000000):
        # bear port and baudrate
        self.BEAR_port = port
        self.BEAR_baudrate = baudrate

        # bear manager
        self.BEAR_controller = BEARController(self.BEAR_port, self.BEAR_baudrate)

        # bear operating mode
        self.BEAR_mode  = 2
        self.BEAR_modes = {'torque': 0, 'velocity': 1, 'position': 2, 'force': 3}

        # bear torque mode
        self.torque_mode  = 0
        self.torque_modes = {'disable': 0, 'enable': 1, 'error': 2, 'damping': 3}

        # error status
        self.error_status = {'Communication': 0, 'Overheat': 1, 'Absolute Position': 2, 'E-STOP': 3, 'Joint Limit': 4, 'Hardware Fault': 5, 'Initialization': 6}

        # bear info
        self.QM  = np.zeros(10)  # bear position array
        self.dQM = np.zeros(10)  # bear velocity array
        self.IQ  = np.zeros(10)  # bear iq array

        # joint info
        self.Q   = np.zeros(10)  # joint position array
        self.dQ  = np.zeros(10)  # joint velocity array
        self.Tau = np.zeros(10)  # joint torque array
        
        # misc
        self.communication_fail_count = 0
        self.torque_mode_id = 0

    def close(self):
        """
        Close PyBEAR serial port
        """
        self.BEAR_controller.pbm.close()

    def start_drivers(self):
        """
        Ping all BEARs and then config their PIDs, limits, etc.
        """
        error = False
        for bear_id in BEAR_LIST:
            trail = 0
            check = False
            print("Pinging BEAR %02d..." % bear_id)
            while not check:
                ping_rtn = self.BEAR_controller.pbm.ping(bear_id)
                time.sleep(0.01)
                if bool(ping_rtn[0]):
                    check = True
                    print("Pinging BEAR %02d Succeed." % bear_id)
                    if ping_rtn[0][1] != 128:  # ERROR!!!
                        print(colored(("BEAR %02d ERROR CODE %d: " + self.decode_error(ping_rtn[0][1])) % (bear_id, ping_rtn[0][1]), 'red'))
                        if ping_rtn[0][1] >> 3 & 1:  # E-Stop Error
                            error = True
                            print(colored("Please release E-Stop button and try again.", 'yellow'))
                            break
                        else:
                            print("Try Clearing Error...")
                            for _ in range(6):
                                self.BEAR_controller.torque_enable(bear_id, 0)
                                time.sleep(0.01)
                                ping_rtn = self.BEAR_controller.pbm.ping(bear_id)
                                time.sleep(0.01)
                                if ping_rtn[0][1] == 128:
                                    print("Clearing Error Succeed.")
                                    break
                            if ping_rtn[0][1] != 128:  # still ERROR!!!
                                error = True
                                print(colored("Clearing Error Failed!", 'red'))
                else:
                    trail += 1
                    if trail > 6:
                        print(colored("BEAR %02d Offline!" % bear_id, 'red'))
                        error = True
                        break
                    time.sleep(0.5)
            print("")

        if error:
            raise BEAR_ERROR_Exception

        self.BEAR_controller.start_drivers()

    def decode_error(self, error_code):
        """
        Decode BEAR error code
        """
        msg = ''
        if error_code >> 7 != 1:
            print(colored("Invalid Error Code!!!", 'red'))
            return msg
        else:
            error_num = 0
            for idx in range(7):
                if error_code >> idx & 1:
                    error_num += 1
                    if error_num > 1:
                        msg += ' & '
                    msg += list(self.error_status.keys())[list(self.error_status.values()).index(idx)]
            if error_num:
                return msg
            else:
                return 'No Error!'

    def torque_enable_all(self, val):
        """
        Torque enable
        """
        self.torque_mode = self.torque_modes['disable'] if val == 0 else self.torque_modes['enable']
        self.BEAR_controller.torque_enable_all(val)

    def set_damping_mode(self):
        """
        Set the joints into damping mode
        """
        self.BEAR_controller.set_damping_mode()

    def get_torque_mode(self):
        """
        Get BEAR torque mode
        """
        try:
            self.torque_mode = self.BEAR_controller.get_torque_mode(BEAR_LIST[self.torque_mode_id])
            self.torque_mode_id = 0 if self.torque_mode_id == 9 else self.torque_mode_id + 1
            self.communication_fail_count = 0
        except:
            self.communication_fail_count += 1

    def is_damping_mode(self):
        """
        Check if in damping mode
        """
        return True if self.torque_mode == 3 else False

    def update_present_temperature_and_voltage(self):
        """
        Only get knee temperature and voltage
        """
        try:
            info = self.BEAR_controller.pbm.bulk_read([BEAR_KNEE_R, BEAR_KNEE_L], ['input_voltage', 'winding_temperature'])
            vol  = min(info[0][0][0], info[1][0][0])
            temp = max(info[0][0][1], info[1][0][1])

            if vol < 14.5:
                print(colored("BEAR Low Voltage!", 'yellow'))

            if temp > 75.0:
                print(colored("BEAR High Temperature!", 'yellow'))

            MM.LEG_STATE.set({'temperature': np.array([temp]),
                              'voltage':     np.array([vol])}, opt='update')
            self.communication_fail_count = 0
        except:
            self.communication_fail_count += 1

    def set_operating_mode(self, mode):
        """
        Set BEAR operating mode
        """
        no_error = True
        if mode == 2:
            bear_mode = 'position'
        elif mode == 1:
            bear_mode = 'velocity'
        elif mode == 0:
            bear_mode = 'torque'
        elif mode == 3:
            bear_mode = 'force'
        else:
            no_error = False

        if no_error:
            print("Changing BEAR mode to %s!" % bear_mode)
            self.BEAR_controller.set_mode(bear_mode)
            self.BEAR_mode = mode
        else:
            print(colored("Invalid BEAR Operation Mode!!!", 'red'))

    def read_write_joint(self, positions=None, velocities=None, torques=None):
        """
        Get joint states and set joint commands
        """
        read_name_list = ['present_position', 'present_velocity', 'present_iq']
        write_data_list = [[]] * 10
        if self.BEAR_mode == 2:  # position
            pos_data = np.concatenate((kin.joint2bear(+1, 0, positions[0:5]), kin.joint2bear(-1, 0, positions[5:10])))
            write_name_list = ['goal_position']
            for idx in range(10):
                write_data_list[idx] = [pos_data[idx]]
        elif self.BEAR_mode == 1:  # velocity
            vel_data = np.concatenate((kin.joint2bear(+1, 1, velocities[0:5]), kin.joint2bear(-1, 1, velocities[5:10])))
            write_name_list = ['goal_velocity']
            for idx in range(10):
                write_data_list[idx] = [vel_data[idx]]
        elif self.BEAR_mode == 0:  # torque
            iq_data = np.concatenate((kin.joint2bear(+1, 2, torques[0:5]), kin.joint2bear(-1, 2, torques[5:10])))
            write_name_list = ['goal_iq']
            for idx in range(10):
                write_data_list[idx] = [iq_data[idx]]
        elif self.BEAR_mode == 3:  # force
            pos_data = np.concatenate((kin.joint2bear(+1, 0, positions[0:5]),  kin.joint2bear(-1, 0, positions[5:10])))
            vel_data = np.concatenate((kin.joint2bear(+1, 1, velocities[0:5]), kin.joint2bear(-1, 1, velocities[5:10])))
            iq_data  = np.concatenate((kin.joint2bear(+1, 2, torques[0:5]),    kin.joint2bear(-1, 2, torques[5:10])))
            write_name_list = ['goal_position', 'goal_velocity', 'goal_iq']
            for idx in range(10):
                iq_data[idx] = MF.sat(iq_data[idx], -IQ_MAX+0.1, +IQ_MAX-0.1)
                write_data_list[idx] = [pos_data[idx], vel_data[idx], iq_data[idx]]

        try:
            info = self.BEAR_controller.pbm.bulk_read_write(BEAR_LIST, read_name_list, write_name_list, write_data_list)
            for idx in range(10):
                self.QM[idx]  = MF.exp_filter(self.QM[idx],  info[idx][0][0], 0.90)
                self.dQM[idx] = MF.exp_filter(self.dQM[idx], info[idx][0][1], 0.90)
                self.IQ[idx]  = MF.exp_filter(self.IQ[idx],  info[idx][0][2], 0.90)

            self.Q[0:5]    = kin.bear2joint(+1, 0, self.QM[0:5])
            self.Q[5:10]   = kin.bear2joint(-1, 0, self.QM[5:10])
            self.dQ[0:5]   = kin.bear2joint(+1, 1, self.dQM[0:5])
            self.dQ[5:10]  = kin.bear2joint(-1, 1, self.dQM[5:10])
            self.Tau[0:5]  = kin.bear2joint(+1, 2, self.IQ[0:5])
            self.Tau[5:10] = kin.bear2joint(-1, 2, self.IQ[5:10])

            data = {'joint_positions':  self.Q,
                    'joint_velocities': self.dQ,
                    'joint_torques':    self.Tau}
            MM.LEG_STATE.set(data)

            self.communication_fail_count = 0

            return True
        except:
            print(colored('Communication Fails!!!', 'red'))
            self.communication_fail_count += 1
            if self.communication_fail_count > 9:
                self.set_damping_mode()
                raise BEAR_ERROR_Exception

            return False
        

def main_loop():
    # Start motors
    bc.start_drivers()
    bc.set_operating_mode(bc.BEAR_modes['torque'])

    commands = {'BEAR_enable':  np.array([0]),
                'BEAR_mode':    np.array([0]),
                'goal_torques': np.zeros(10)}
    MM.LEG_COMMAND.set(commands)

    read_write_frequency   = 1000  # read/write at 1000 Hz
    damping_check_freq     = 10    # check damping at 10 Hz
    temp_vol_update_freq   = 0.1   # update temperature and voltage at 0.1 Hz
    check_thread_frequency = 50    # check thread error at 50 Hz

    read_write_duration      = 1. / read_write_frequency
    damping_check_duration   = 1. / damping_check_freq
    temp_vol_update_duration = 1. / temp_vol_update_freq
    check_thread_duration    = 1. / check_thread_frequency

    print("====== The BEAR Actuator Communication Thread is running at", read_write_frequency, "Hz... ======")

    last_read_write_time      = time.time()
    last_damping_check_time   = time.time()
    last_temp_vol_update_time = time.time()
    last_check_thread_time    = time.time()

    thread_run = False
    t0 = time.time()
    while True:
        loop_start_time = time.time()
        elapsed_time    = loop_start_time - t0

        if not thread_run and elapsed_time > 1:
            print(colored('THREAD IS DOING GREAT!', 'green'))
            MM.THREAD_STATE.set({'bear': np.array([1])}, opt='update')  # thread is running
            thread_run = True
        
        # check threading error
        last_check_thread_elapse = loop_start_time - last_check_thread_time
        if last_check_thread_elapse > check_thread_duration:
            last_check_thread_time = loop_start_time
            if Bruce.thread_error():
                Bruce.stop_threading()
        
        # check BEAR damping
        last_damping_check_elapse = loop_start_time - last_damping_check_time
        if last_damping_check_elapse > damping_check_duration:
            last_damping_check_time = loop_start_time
            bc.get_torque_mode()
            if bc.is_damping_mode():
                raise BEAR_ESTOP_Exception
            elif bc.torque_mode != int(commands['BEAR_enable'][0]):
                print(colored("Error Enable Status!", 'red'))
                raise BEAR_ERROR_Exception

        # update temperature & voltage
        last_temp_vol_update_elapse = loop_start_time - last_temp_vol_update_time
        if last_temp_vol_update_elapse > temp_vol_update_duration:
            last_temp_vol_update_time = loop_start_time
            bc.update_present_temperature_and_voltage()

        # read/write BEAR
        last_read_write_elapse = loop_start_time - last_read_write_time
        if last_read_write_elapse > read_write_duration:
            last_read_write_time = loop_start_time

            commands = MM.LEG_COMMAND.get()

            if int(commands['BEAR_mode'][0]) != bc.BEAR_mode:
                bc.set_operating_mode(int(commands['BEAR_mode'][0]))

            bc.read_write_joint(positions=commands['goal_positions'], velocities=commands['goal_velocities'], torques=commands['goal_torques'])

            if int(commands['BEAR_enable'][0]) != bc.torque_mode:
                bc.torque_enable_all(int(commands['BEAR_enable'][0]))

            if last_read_write_elapse > read_write_duration * 1.5:
                print(colored('Delayed ' + str(1e3 * (last_read_write_elapse - read_write_duration))[0:5] + ' ms at T = ' + str(elapsed_time)[0:5] + ' s', 'yellow'))


if __name__ == "__main__":
    # BRUCE SETUP
    Bruce = RDS.BRUCE()

    # BEAR SETUP
    bc = bear_controller(port=BEAR_port, baudrate=BEAR_baudrate)
    
    try:
        main_loop()
    except BEAR_ERROR_Exception:
        print(colored('BEAR IN ERROR! Terminate Now!', 'red'))
        MM.THREAD_STATE.set({'bear': np.array([3])}, opt='update')  # BEAR in error
    except BEAR_ESTOP_Exception:
        print(colored('BEAR IN E-STOP! Terminate Now!', 'red'))
        MM.THREAD_STATE.set({'bear': np.array([4])}, opt='update')  # BEAR ESTOP
    except NameError:
        print(colored('THREAD STOPPED PEACEFULLY!', 'light_grey'))
        MM.THREAD_STATE.set({'bear': np.array([0])}, opt='update')  # thread is stopped
    except (Exception, KeyboardInterrupt) as error:
        print(error)
        print(colored('THREAD IN ERROR! TERMINATE NOW!', 'red'))
        MM.THREAD_STATE.set({'bear': np.array([2])}, opt='update')  # thread in error
    finally:
        bc.set_damping_mode()